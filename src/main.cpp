#include <algorithm>
#include <iostream>
#include <ranges>

int main(int argc, char *argv[]) {
  std::ranges::for_each(std::ranges::views::iota(0, 10), [](const int &index) {
    std::cout << index << std::endl;
  });
}
