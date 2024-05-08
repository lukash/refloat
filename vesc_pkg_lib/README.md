# VESC Package Library

This is a VESC Package library containing the necessary files to build a VESC Package. It is intended to be included in a VESC Package repository as a `git subtree` in the `vesc_pkg_lib` directory.

## Adding the library to a repository

```bash
git subtree add --prefix vesc_pkg_lib https://github.com/lukash/vesc_pkg_lib.git main --squash
```

## Updating the library in a repository to a new version

```bash
git subtree pull --prefix vesc_pkg_lib https://github.com/lukash/vesc_pkg_lib.git main --squash
```
