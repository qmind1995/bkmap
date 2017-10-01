//
// Created by tri on 21/09/2017.
//

#ifndef BKMAP_SQLITE3_UTILS_H
#define BKMAP_SQLITE3_UTILS_H

#include <cstdio>
#include <cstdlib>
#include <string>

#include "ext/SQLite/sqlite3.h"

namespace bkmap {

    inline int SQLite3CallHelper(const int result_code, const std::string& filename,
                                 const int line_number) {
        switch (result_code) {
            case SQLITE_OK:
            case SQLITE_ROW:
            case SQLITE_DONE:
                return result_code;
            default:
                fprintf(stderr, "SQLite error [%s, line %i]: %s\n", filename.c_str(),
                        line_number, sqlite3_errstr(result_code));
                exit(EXIT_FAILURE);
        }
    }

#define SQLITE3_CALL(func) SQLite3CallHelper(func, __FILE__, __LINE__)

#define SQLITE3_EXEC(database, sql, callback)                                 \
  {                                                                           \
    char* err_msg = nullptr;                                                  \
    int rc = sqlite3_exec(database, sql, callback, nullptr, &err_msg);        \
    if (rc != SQLITE_OK) {                                                    \
      fprintf(stderr, "SQLite error [%s, line %i]: %s\n", __FILE__, __LINE__, \
              err_msg);                                                       \
      sqlite3_free(err_msg);                                                  \
    }                                                                         \
  }

}  // namespace bkmap

#endif //BKMAP_SQLITE3_UTILS_H
