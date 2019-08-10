@@
expression x, y;
statement S;
@@
  x = mbedtls_malloc(...);
  y = mbedtls_malloc(...);
  ...
* if (x == NULL || y == NULL)
    S

@@
expression x, y;
statement S;
@@
  if (
*   (x = mbedtls_malloc(...)) == NULL
    ||
*   (y = mbedtls_malloc(...)) == NULL
  )
    S
