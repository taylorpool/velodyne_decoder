{ pkgs ? import ./version.nix }:
pkgs.callPackage ./derivation.nix { stdenv = pkgs.llvmPackages_16.stdenv; clang-tools = pkgs.clang-tools_16; }
