{ pkgs ? import ./version.nix }:
pkgs.callPackage ./derivation.nix { stdenv = pkgs.llvmPackages_16.stdenv; }
