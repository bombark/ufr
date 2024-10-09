// include the PHP API itself
#include <php.h>
#include <ufr.h>

// then include the header of your extension
#include "php_ufr.h"


ZEND_BEGIN_ARG_INFO_EX(ufr_publisher_args, 0, 0, 1)
    ZEND_ARG_TYPE_INFO(0, identifier, IS_STRING, 0)
ZEND_END_ARG_INFO()

// register our function to the PHP API 
// so that PHP knows, which functions are in this module
zend_function_entry ufr_php_functions[] = {
    PHP_FE(ufr_publisher, ufr_publisher_args)
    {NULL, NULL, NULL}
};

// some pieces of information about our module
zend_module_entry ufr_php_module_entry = {
    STANDARD_MODULE_HEADER,
    PHP_UFR_EXTNAME,
    ufr_php_functions,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    PHP_UFR_VERSION,
    STANDARD_MODULE_PROPERTIES
};

// use a macro to output additional C code, to make ext dynamically loadable
ZEND_GET_MODULE(ufr_php)


// Finally, we implement our "Hello World" function
// this function will be made available to PHP
// and prints to PHP stdout using printf
PHP_FUNCTION(ufr_publisher) {
    char *ufr_params = "";
    size_t ufr_params_len = 0;

    ZEND_PARSE_PARAMETERS_START(1, 1)
    Z_PARAM_STRING(ufr_params, ufr_params_len)
    ZEND_PARSE_PARAMETERS_END();

    link_t link = ufr_publisher(ufr_params);
    // php_printf("Hello World! (from our extension) %s a\n", var);
}

