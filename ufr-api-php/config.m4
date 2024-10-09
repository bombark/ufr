PHP_ARG_WITH(php_ufr, Whether to enable the UFR PHP extension, [ --enable-ufr-php Enable UFR PHP])

if test "$PHP_UFR" != "no"; then

    LIBNAME=ufr
    LIBSYMBOL=ufr_publisher
    LIBOTHERLIB_LIBS=-lufr

    PHP_CHECK_LIBRARY($LIBNAME,$LIBSYMBOL,
    [
        PHP_ADD_LIBRARY($LIBNAME,1,EXTRA_CFLAGS)
        PHP_SUBST(EXTRA_CFLAGS)
        AC_DEFINE(HAVE_MYEXTENSION, 1, [Whether you have MyExtension])
    ],[
        AC_MSG_ERROR([wrong lib$LIBNAME version or library not found])
    ])

    PHP_NEW_EXTENSION(php_ufr, php_ufr.c, $ext_shared)
fi
