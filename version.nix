import ( builtins.fetchTarball "https://github.com/NixOS/nixpkgs/archive/refs/tags/23.05.tar.gz" ) {
  overlays = [
    ( import ( builtins.fetchGit {
      url = "git@github.com:taylorpool/tpool_nixpkgs.git";
      ref = "main";
      rev = "2a8222b13e0d0ce4e56cff94b9df6cbb910c8c40";
      } ) )
    ( import ( ( builtins.fetchTarball "https://github.com/lopsided98/nix-ros-overlay/archive/master.tar.gz"
      ) + "/overlay.nix" ) )
  ];
}
