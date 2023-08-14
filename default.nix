{ pkgs ? import ./version.nix }:
pkgs.callPackage ./derivation.nix {}
