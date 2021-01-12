#ifndef H_LOGGING_
#define H_LOGGING_

#include <iostream>

#define ERROR_LOG(X) printf("\033[31m[ERROR] %s\033[0m\n", X)
#define INFO_LOG(X) printf("[INFO] %s\n", X)
#define WARNING_LOG(X) printf("\033[33m[WARNING] %s\033[0m\n", X)

#endif