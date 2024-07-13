{
  description = "Modm Flake";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = {
    self,
    nixpkgs,
    flake-utils,
    ...
  }:
    flake-utils.lib.eachDefaultSystem (
      system: let
        pkgs = import nixpkgs {
          inherit system;
        };
        lbuild-python-package = pkgs.python311Packages.buildPythonPackage rec {
          pname = "lbuild";
          version = "1.21.8";
          src = pkgs.python311Packages.fetchPypi {
            inherit pname version;
            sha256 = "70abaf36b46a239c1ee4fa9c71e843381b21b9dccede4cd50df392ae78560e57";
          };
          propagatedBuildInputs = [
            pkgs.python311Packages.lxml
            pkgs.python311Packages.jinja2
            pkgs.python311Packages.anytree
            pkgs.python311Packages.testfixtures
            pkgs.python311Packages.coverage
            pkgs.python311Packages.gitpython
            pkgs.python311Packages.pip
          ];
        };
        modm-python-package = pkgs.python311Packages.buildPythonPackage rec {
          pname = "modm";
          version = "0.1.2";
          src = pkgs.python311Packages.fetchPypi {
            inherit pname version;
            sha256 = "4c3edbf7fa945d3fc10c0191d1000ee5b491f3d0e43f363af3b9de12e38a5d12";
          };
          propagatedBuildInputs = [
            lbuild-python-package
            pkgs.python311Packages.lxml
            pkgs.python311Packages.pyelftools
            pkgs.python311Packages.pip
          ];
        };
      in
        with pkgs; {
          devShells.default = mkShell {
            buildInputs = [
              gcc
              scons
              openocd
              lldb
              gcc-arm-embedded
              (python311.withPackages (python-pkgs: [
                python-pkgs.pandas
                python-pkgs.pygobject3
                python-pkgs.numpy
                python-pkgs.pip
                python-pkgs.matplotlib
                modm-python-package
                lbuild-python-package
              ]))
            ];
          };
        }
    );
}
