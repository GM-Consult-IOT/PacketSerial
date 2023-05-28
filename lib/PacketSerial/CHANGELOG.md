<!-- PacketSerial -->

## 0.0.8
### BREAKING CHANGES
* Changed signature of `read` function.

## 0.0.7+1
* Added `ps_header_t header()` method to `PS_BYTE_ARRAY` struct.

## 0.0.7
### BREAKING CHANGES
* Removed struct `PS_FRAME`.
* Changed state machine in `serial_rx`.

## 0.0.6+3

* Bug fixes.

## 0.0.6+3
### BREAKING CHANGES
* Moved all configuration data to `ps_config.h`.

## 0.0.6+2

* Removed serial debug code.
## 0.0.6+1

### BREAKING CHANGES
* Added private variables for ps_config settings to allow easy configuration using `#define` statements.

## 0.0.6

### BREAKING CHANGES
* Changed structure of #defines and consts to fix usses with `PS_DEBUG` usage.
* Added `print` methods to `PS_BYTE_ARRAY` and `PS_FRAME`.

## 0.0.5+1

### BREAKING CHANGES:
* Changed value of `PS_ERR_START_UP_FAIL` to 0xff.
* Added `setBitValues` method.
* Re-organized dependencies and defines

## 0.0.5

### BREAKING CHANGES:
* Changed return value of `onStartup` method.

## 0.0.4

### BREAKING CHANGES:
* Changed signature of `onError` method to take `uint8_t` as parameter `error`.
* Changed scope of `onError` method to protected.

## 0.0.3

### BREAKING CHANGES:
* Changed signature of `onSerialRx` and `onSerialTx`.

## 0.0.2+2

Minor edits

## 0.0.2+1

Minor edits

## 0.0.2

Minor edits

## 0.0.1+2

Minor edits

## 0.0.1+1

Minor edits

## 0.0.1

* Initial version.