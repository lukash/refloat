{
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-25.11";
    flake-utils.url = "github:numtide/flake-utils";
    vesc-tool.url = "github:vedderb/vesc_tool/release_6_06";
    vesc-tool.inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs = {
    self,
    flake-utils,
    nixpkgs,
    vesc-tool,
  } @ inputs:
    flake-utils.lib.eachSystem ["x86_64-linux"] (
      system: let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [
            (final: prev: {
              vesc-tool = vesc-tool.packages.${system}.vesc-tool;
            })
            (import ./nix/overlay.nix)
            (final: prev: {
              refloat = prev.refloat.overrideDerivation (old: {
                version = prev.lib.strings.removeSuffix "\n" (builtins.readFile ./version);
                src = self;

                makeFlags = [
                  "GIT_HASH=${builtins.substring 0 8 (self.rev or self.dirtyRev)}"
                  "BUILD_DATE=${self.lastModifiedDate}"
                ];
              });
            })
          ];
        };
      in {
        packages =
          pkgs
          // {
            default = pkgs.refloat;
          };
      }
    );
}
