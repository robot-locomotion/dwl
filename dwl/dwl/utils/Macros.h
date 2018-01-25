#include <cstdio>
#include <iostream>

#define RED     "\033[1m\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[1m\033[33m"
#define BLUE    "\033[1m\033[34m"
#define MAGENTA "\x1b[35m"
#define CYAN    "\x1b[36m"
#define COLOR_RESET   "\x1b[0m"
#define PRINT_SCALAR(x) std::cout << #x " = " << x << "\n" << std::endl
#define PRINT_VECTOR(x) std::cout << #x " = ";\
	for (int i = 0; i < x.size(); i++) {std::cout << x[i] << " ";}\
	std::cout << std::endl;
