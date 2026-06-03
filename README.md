# ROS — Register Optimizations with Safety

A header-only C++20 library for safe, optimized hardware register operations.

ROS encodes register layout and access semantics into the type system so that
field-level reads and writes are checked at compile time. When a register
contains fields with special hardware behavior (clear-on-write-1, toggle,
read-clear, etc.), the library automatically computes identity values and
optimizes Read-Modify-Write sequences to avoid unintended side effects.

## Features

- **Compile-time safety** — field widths, bit positions, and access permissions
  are validated at compile time. Writing a read-only field or reading a
  write-only register is a compilation error.
- **IEEE 1685 access types** — all 12 standard access types (RW, RO, WO,
  RC, RS, RW\_0C, RW\_1C, RW\_0S, RW\_1S, RW\_1T, RW\_O, RSVD) with correct
  identity and mask semantics.
- **RMW optimization** — the library skips the bus read when all
  Read-Modify-Write-able bits are covered by the write, which is critical for
  registers containing RC/RS fields where an unnecessary read would trigger
  hardware side effects.
- **Identity preservation** — partial writes fill non-targeted special fields
  with their hardware no-op value, preventing unintended clears, sets, or
  toggles.
- **Bus abstraction** — a pluggable bus interface decouples register definitions
  from the transport (memory-mapped, SPI, I2C, etc.).
- **Zero overhead** — all dispatch is resolved at compile time; the generated
  code is a minimal sequence of bus reads and writes.

## Quick start

### CMake with [CPM](https://github.com/cpm-cmake/CPM.cmake)

```cmake
CPMAddPackage("gh:rykovv/ros#main")
target_link_libraries(your_target PRIVATE ros)
```

### Include

```cpp
#include <ros.hpp>     // everything
// — or pick what you need —
#include <ros/eval.hpp>
#include <ros/access.hpp>
#include <ros/literals.hpp>
```

## Compiler compatibility

| Standard | GCC        | Clang        | MSVC |
|----------|------------|--------------|------|
| C++20    | 12 thru 16 | 14 thru 21   | —    |

## At a glance

### Define a register

```cpp
#include <ros.hpp>
using namespace ros;
using namespace ros::literals;

// 16-bit control register at address 0x40
struct ctrl_reg : reg<ctrl_reg, uint16_t, 0x40_addr, my_bus> {
    // boiler plate to allow assignment semantics
    using reg<ctrl_reg, uint16_t, 0x40_addr, my_bus>::operator=;

    field<ctrl_reg,  3_msb,  0_lsb, access_type::RW>    mode;
    field<ctrl_reg,  7_msb,  4_lsb, access_type::RO>    status;
    field<ctrl_reg, 11_msb,  8_lsb, access_type::RW_1C> irq_flags;
    field<ctrl_reg, 15_msb, 12_lsb, access_type::WO>    command;
};
```

### Write fields

```cpp
constexpr ctrl_reg cr{};

// Compile-time values (literals)
eval(cr.mode = 0x5_f, cr.command = 0xA_f);

// Runtime values
uint16_t m = read_mode_from_config();
eval(cr.mode = m);
```

### Read fields

```cpp
// Single read — returns the field value directly
auto mode_val = eval(cr.mode);

// Multiple reads — returns a tuple
auto [mode_val, status_val] = eval(cr.mode, cr.status);
```

### Mix reads and writes in one call

```cpp
// Write mode, read back status — single bus transaction
auto status_val = eval(cr.mode = 3_f, cr.status);
```

### Read-modify-write with a lambda

```cpp
// Toggle the low bit of mode
eval(cr.mode([](auto val) { return val ^ 1; }));
```

### Write a whole register

```cpp
eval(cr = 0x1234_r);   // compile-time value
eval(cr = some_var);    // runtime value
```

## Access types

ROS supports all access types defined by the IEEE 1685 (IP-XACT) and
ARM CMSIS-SVD standards. Each type carries distinct read, write, and identity
semantics that the library enforces automatically.

| Type    | Description             | Read | Write | Identity* | RMW safe* |
|---------|-------------------------|------|-------|-----------|-----------|
| `RW`    | Read-Write              | yes  | yes   | `0`       | yes       |
| `RO`    | Read-Only               | yes  | —     | —         | —         |
| `WO`    | Write-Only              | —    | yes   | —         | —         |
| `RC`    | Read-Clear              | yes  | —     | —         | —         |
| `RS`    | Read-Set                | yes  | —     | —         | —         |
| `RW_1C` | Clear-on-Write-1        | yes  | yes   | `0`       | no        |
| `RW_0C` | Clear-on-Write-0        | yes  | yes   | `mask`    | no        |
| `RW_1S` | Set-on-Write-1          | yes  | yes   | `0`       | no        |
| `RW_0S` | Set-on-Write-0          | yes  | yes   | `mask`    | no        |
| `RW_1T` | Toggle-on-Write-1       | yes  | yes   | `0`       | no        |
| `RW_O`  | Read-Write-Once         | yes  | yes   | `0`       | no        |
| `RSVD`  | Reserved                | —    | —     | —         | —         |

***Identity** is the value that, when written to a field, produces no hardware
side effect. For example, writing `0` to an `RW_1C` field does not clear
anything; writing all-ones to an `RW_0S` field does not set anything.

***RMW safe** indicates whether a field participates in the `rmw_mask`. Only
plain `RW` fields do — all other writable types use their identity value during
partial writes instead of a read-back value.

## How eval() works

`eval()` is the single entry point for all register I/O. It accepts any
combination of field reads, compile-time writes, runtime writes, and
lambda-based read-modify-write operations — all targeting the same register.

```
eval(field_ops...)
  │
  ├─ separate into: CT writes, RT writes, invocable writes, reads
  │
  ├─ compute write_mask (union of all written field masks)
  │
  ├─ if write_mask != 0:
  │   ├─ start value = register identity
  │   │
  │   ├─ if partial write (write_mask doesn't cover rmw_mask):
  │   │   └─ bus read → merge RMW bits into value
  │   │
  │   ├─ apply invocable writes (using original field values)
  │   ├─ apply CT writes
  │   ├─ apply RT writes
  │   └─ bus write
  │
  └─ else (read-only):
      └─ bus read
  │
  └─ extract and return read field values
```

### Why identity matters

Consider a register with an `RW` data field and an `RW_1C` interrupt-status
field. You want to update `data` without accidentally clearing pending
interrupts:

```cpp
struct irq_reg : reg<irq_reg, uint8_t, 0x10_addr, my_bus> {
    using reg<irq_reg, uint8_t, 0x10_addr, my_bus>::operator=;

    field<irq_reg, 3_msb, 0_lsb, access_type::RW>    data;
    field<irq_reg, 7_msb, 4_lsb, access_type::RW_1C> irq_status;
};

constexpr irq_reg r{};
eval(r.data = 0xA_f);
```

A naive RMW would read back `irq_status = 0xF` (interrupts pending) and
write it back, clearing them all. ROS avoids this: `irq_status` gets its
identity value (`0`), which is a no-op for `RW_1C`. The bus read is skipped
entirely because `data` alone covers the full `rmw_mask`.

### Bus read avoidance

The library only issues a bus read when it must preserve plain `RW` bits that
the current `eval()` call does not write. This is tracked by `rmw_mask` (the
union of all plain `RW` field masks). When the written fields cover `rmw_mask`
completely, no read is needed — even if the register has other writable fields
like `RW_1C` or `RW_0S`.

This optimization is essential for registers containing `RC` (Read-Clear)
fields: reading such a register as part of an RMW would clear interrupt flags
as a side effect.

## Bus interface

Implement a bus struct with `read` and `write` static methods:

```cpp
struct my_bus {
    template <typename T, typename Addr>
    static T read(Addr address) {
        return *reinterpret_cast<volatile T*>(address);
    }

    template <typename T, typename Addr>
    static void write(T val, Addr address) {
        *reinterpret_cast<volatile T*>(address) = val;
    }
};
```

For testing, use a mock bus that logs operations:

```cpp
struct mock_bus {
    template <typename T, typename Addr>
    static T read(Addr address) {
        // log the read, return a configurable value
    }

    template <typename T, typename Addr>
    static void write(T val, Addr address) {
        // log the write
    }
};
```

## User-defined literals

All literals live in `ros::literals`:

| Literal | Type | Example |
|---------|------|---------|
| `_f`    | Compile-time field value | `0xA_f` |
| `_r`    | Compile-time register value | `0xFF_r` |
| `_msb`  | Most-significant bit position | `7_msb` |
| `_lsb`  | Least-significant bit position | `0_lsb` |
| `_addr` | Register address | `0x40_addr` |

## Compile-time safety examples

```cpp
// Error: cannot write read-only field
eval(cr.status = 1_f);       // static_assert failure

// Error: value exceeds field width (4 bits → max 0xF)
eval(cr.mode = 0xFF_f);      // static_assert failure

// Error: cannot read write-only register
eval(wo_register);           // static_assert failure

// Error: field assigned twice in one eval
eval(cr.mode = 1_f, cr.mode = 2_f);  // concept constraint failure
```

## Unsafe operations

For cases where you need to bypass safety checks (e.g., writing a known-good
raw value to a field with read-only bits):

```cpp
// Bypasses the runtime bounds check
eval(cr.mode.unsafe = raw_value);
```

## Running tests

```bash
cmake -Bbuild -GNinja
ninja -C build
ctest --test-dir build/test
```

## License

[MIT](LICENSE)
