// ============================================================================
//  Header
// ============================================================================

#include <ufr.h>
#include <php.h>
#include "php_ufr.h"

// Parameters of each PHP function
ZEND_BEGIN_ARG_INFO_EX(ufr_publisher_args, 0, 0, 1)
    ZEND_ARG_TYPE_INFO(0, identifier, IS_STRING, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_INFO_EX(ufr_put_args, 0, 0, 2)
    ZEND_ARG_TYPE_INFO(0, identifier, IS_LONG, 0)
    ZEND_ARG_TYPE_INFO(0, identifier, IS_STRING, 0)
ZEND_END_ARG_INFO()

// register our function to the PHP API 
// so that PHP knows, which functions are in this module
zend_function_entry ufr_php_functions[] = {
    PHP_FE(ufr_publisher, ufr_publisher_args)
    PHP_FE(ufr_put, ufr_put_args)
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

link_t g_links[10];

// ============================================================================
//  Private Functions
// ============================================================================

int alloc_link_id() {
    static int i = 0;
    i = i+1;
    return i;
}

// ============================================================================
//  Public PHP Functions
// ============================================================================

// use a macro to output additional C code, to make ext dynamically loadable
ZEND_GET_MODULE(ufr_php)

PHP_FUNCTION(ufr_publisher) {
    char *ufr_params = "";
    size_t ufr_params_len = 0;

    ZEND_PARSE_PARAMETERS_START(1, 1)
    Z_PARAM_STRING(ufr_params, ufr_params_len)
    ZEND_PARSE_PARAMETERS_END();

    int link_id = alloc_link_id();
    link_t* link = &g_links[link_id];
    *link = ufr_publisher(ufr_params);
    // php_printf("Hello World! (from our extension) %s a\n", var);

    RETURN_LONG(link_id);
}

PHP_FUNCTION(ufr_put) {
    // prepare the variable for PHP parameters
    size_t num_args = ZEND_NUM_ARGS();
    long int link_id = 0;
    char* link_put_format = NULL;
    size_t link_put_format_len = 0;

    // get the PHP parameters
    ZEND_PARSE_PARAMETERS_START(2, num_args)
    Z_PARAM_LONG(link_id)
    Z_PARAM_STRING(link_put_format, link_put_format_len)
    ZEND_PARSE_PARAMETERS_END();

    // Execute the function
    link_t* link = &g_links[link_id];
    ufr_put(link, link_put_format, "aaa", "dddd");
    // php_printf("Hello World! (from our extension) %s a\n", var);

    // Return the result
    RETURN_LONG(0);
}

