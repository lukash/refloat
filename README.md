# Refloat - VESC Package
Refloat is a VESC Package for self-balancing skateboards. It aims to:
- Provide a polished and full-featured user experience
- Maintain a clean and reliable codebase that is easy to extend
- Make it easy for anyone to use Refloat as a base for experimentation
- Standardize interfaces so that package clones can interact with other parts of the ecosystem (3rd party apps, light modules, VESC Express addons etc.) in a compatible way

_**If you're looking for README of the actual package, you can find it [here](package_README.md).**_

## Contributing
Contributions are welcome and appreciated, please refer to [Contributing](CONTRIBUTING.md).

## Building
### Requirements
- `gcc-arm-embedded` version 13 or higher
- `make`
- `vesc_tool`

To build the package, run:
```sh
make
```

If you don't have `vesc_tool` in your `$PATH` (but you have, for example, a downloaded `vesc_tool` binary), you can specify the `vesc_tool` to use:
```sh
make VESC_TOOL=/path/to/vesc_tool
```
