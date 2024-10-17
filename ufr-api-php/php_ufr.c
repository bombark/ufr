// ============================================================================
//  Header
// ============================================================================

#include <ufr.h>
#include <php.h>
#include "php_ufr.h"

// Parameters of each PHP function
ZEND_BEGIN_ARG_INFO_EX(ufr_open_args, 0, 0, 1)
    ZEND_ARG_TYPE_INFO(0, identifier, IS_STRING, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_INFO_EX(ufr_close_args, 0, 0, 1)
    ZEND_ARG_TYPE_INFO(0, identifier, IS_LONG, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_INFO_EX(ufr_put_args, 0, 0, 2)
    ZEND_ARG_TYPE_INFO(0, identifier, IS_LONG, 0)
    ZEND_ARG_TYPE_INFO(0, identifier, IS_STRING, 0)
ZEND_END_ARG_INFO()

ZEND_BEGIN_ARG_INFO_EX(ufr_get_args, 0, 0, 2)
    ZEND_ARG_TYPE_INFO(0, identifier, IS_LONG, 0)
    ZEND_ARG_TYPE_INFO(0, identifier, IS_STRING, 0)
ZEND_END_ARG_INFO()

// register our function to the PHP API 
// so that PHP knows, which functions are in this module
zend_function_entry ufr_php_functions[] = {
    PHP_FE(ufr_publisher, ufr_open_args)
    PHP_FE(ufr_subscriber, ufr_open_args)
    PHP_FE(ufr_client, ufr_open_args)
    PHP_FE(ufr_server, ufr_open_args)
    PHP_FE(ufr_close, ufr_close_args)
    PHP_FE(ufr_put, ufr_put_args)
    PHP_FE(ufr_get, ufr_get_args)
    PHP_FE_END
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

#define LINK_MAX   32
link_t* g_links[LINK_MAX];

// ============================================================================
//  Private Functions
// ============================================================================

static
uint16_t alloc_link_id() {
    // get free slot
    uint16_t pos;
    for (uint16_t i=0; i<LINK_MAX; i++) {
        if ( g_links[i] == NULL ) {
            return i;
        }
    }
    return (uint16_t) -1;
}

link_t* malloc_link(const uint16_t id) {
    // check parameters is ok
    if ( id >= LINK_MAX ) {
        return NULL;
    }
    if ( g_links[id] != NULL ) {
        return NULL;
    }
 
    // malloc link and set in the global array
    link_t* link = malloc(sizeof(link_t));
    if ( link == NULL ) {
        return NULL;
    }

    // ufr_init_link(link);
    g_links[id] = link;
    return link;
}

static 
link_t* get_link_by_id(const uint16_t id) {
    if ( id < LINK_MAX )
        return g_links[id];
    return NULL;
}

// ============================================================================
//  Public PHP Functions
// ============================================================================

// use a macro to output additional C code, to make ext dynamically loadable
ZEND_GET_MODULE(ufr_php)

PHP_FUNCTION(ufr_publisher) {
    // prepare the variables for PHP parameters
    char *ufr_params = "";
    size_t ufr_params_len = 0;

    // parse PHP parameters
    ZEND_PARSE_PARAMETERS_START(1, 1)
    Z_PARAM_STRING(ufr_params, ufr_params_len)
    ZEND_PARSE_PARAMETERS_END();

    // alocate and open the link
    uint16_t link_id = alloc_link_id();
    link_t* link = malloc_link(link_id);
    if ( link == NULL ) {
        php_error_docref(NULL, E_WARNING, "Error to allocate a link"); // Mensagem de erro
        RETURN_FALSE;
    }
    *link = ufr_publisher(ufr_params);

    // return the link id
    RETURN_LONG(link_id);
}

PHP_FUNCTION(ufr_subscriber) {
    // prepare the variables for PHP parameters
    char *ufr_params = "";
    size_t ufr_params_len = 0;

    // parse PHP parameters
    ZEND_PARSE_PARAMETERS_START(1, 1)
    Z_PARAM_STRING(ufr_params, ufr_params_len)
    ZEND_PARSE_PARAMETERS_END();

    // alocate and open the link
    uint16_t link_id = alloc_link_id();
    link_t* link = malloc_link(link_id);
    *link = ufr_subscriber(ufr_params);
    
    // return the link id
    RETURN_LONG(link_id);
}

PHP_FUNCTION(ufr_client) {
    // prepare the variables for PHP parameters
    char *ufr_params = "";
    size_t ufr_params_len = 0;

    // parse PHP parameters
    ZEND_PARSE_PARAMETERS_START(1, 1)
    Z_PARAM_STRING(ufr_params, ufr_params_len)
    ZEND_PARSE_PARAMETERS_END();

    // alocate and open the link
    uint16_t link_id = alloc_link_id();
    link_t* link = malloc_link(link_id);
    *link = ufr_client(ufr_params);
    
    // return the link id
    RETURN_LONG(link_id);
}

PHP_FUNCTION(ufr_server) {
// prepare the variables for PHP parameters
    char *ufr_params = "";
    size_t ufr_params_len = 0;

    // parse PHP parameters
    ZEND_PARSE_PARAMETERS_START(1, 1)
    Z_PARAM_STRING(ufr_params, ufr_params_len)
    ZEND_PARSE_PARAMETERS_END();

    // alocate and open the link
    uint16_t link_id = alloc_link_id();
    link_t* link = malloc_link(link_id);
    *link = ufr_client(ufr_params);
    
    // return the link id
    RETURN_LONG(link_id);
}

PHP_FUNCTION(ufr_close) {
    // variable for PHP parameters
    int64_t link_id;

    // get PHP parameters
    ZEND_PARSE_PARAMETERS_START(1, 1)
    Z_PARAM_LONG(link_id);
    ZEND_PARSE_PARAMETERS_END();

    // get the link by ID
    link_t* link = get_link_by_id(link_id);
    if ( link == NULL ) {
        php_error_docref(NULL, E_WARNING, "Link is invalid"); // Mensagem de erro
        RETURN_FALSE;
    }

    // free link block and set NULL to the global array
    ufr_close(link);
    free(link);
    g_links[link_id] = NULL;
}

PHP_FUNCTION(ufr_put) {
    // prepare the variable for PHP parameters
    const size_t num_args = ZEND_NUM_ARGS();
    long int link_id = 0;
    char* link_put_format = "";
    size_t link_put_format_len = 0;
    int varargs_len;
    zval *varargs = NULL;

    // get the parameters
    if (num_args > 0) {
        if ( 
            zend_parse_parameters (
                num_args, "ls*", 
                &link_id, 
                &link_put_format, &link_put_format_len,
                &varargs, &varargs_len
            ) == FAILURE
        ) {
            RETURN_NULL();
        }
    }

    // Get the Link by ID
    link_t* link = get_link_by_id(link_id);
    if ( link == NULL ) {
        php_error_docref(NULL, E_WARNING, "Link is invalid"); // Mensagem de erro
        RETURN_FALSE;
    }

    // execute ufr_put
    for (int i=0, i_v=0; i<link_put_format_len; i++) {
        const char c = link_put_format[i];
        if ( i_v > varargs_len ) {
            break;
        }

        // send Integer
        if ( c == 'i' ) {           
            if ( Z_TYPE(varargs[i_v]) == IS_LONG ) {
                ufr_put(link, "i", Z_LVAL(varargs[i_v]));
            } else if ( Z_TYPE(varargs[i_v]) == IS_STRING ) {
                const char* val_str = Z_STRVAL(varargs[i_v]);
                const int64_t val_i64 = atoi(val_str);
                ufr_put(link, "i", val_i64);
            }
            i_v += 1;

        // send String
        } else if ( c == 's' ) {
            if ( Z_TYPE(varargs[i_v]) == IS_LONG ) {
                char val_str[32];
                const int64_t val_i64 = Z_LVAL(varargs[i_v]);
                snprintf(val_str, 32, "%ld", val_i64);
                ufr_put(link, "s", val_str);
            } else if ( Z_TYPE(varargs[i_v]) == IS_STRING ) {
                ufr_put(link, "s", Z_STRVAL(varargs[i_v]));
            }
            i_v += 1;

        } else if ( c == '\n' ) {
            ufr_put(link, "\n");

        // error
        } else {

        }

    }

    // Return the result
    RETURN_LONG(0);
}

PHP_FUNCTION(ufr_get) {
    // prepare the variable for PHP parameters
    long int link_id = 0;
    char* link_put_format = "";
    size_t link_put_format_len = 0;

    // parse the PHP parameters
    ZEND_PARSE_PARAMETERS_START(2, 2)
    Z_PARAM_LONG(link_id);
    Z_PARAM_STRING(link_put_format, link_put_format_len);
    ZEND_PARSE_PARAMETERS_END();

    // Get the Link by ID
    link_t* link = get_link_by_id(link_id);
    if ( link == NULL ) {
        php_error_docref(NULL, E_WARNING, "Link is invalid"); // Mensagem de erro
        RETURN_FALSE;
    }

    // Execute the function
    int count = 0;
    array_init(return_value);
    for (int i=0, i_v=0; i<link_put_format_len; i++) {
        const char c = link_put_format[i];

        if ( c == '^' ) {
            if ( ufr_recv(link) != UFR_OK ) {
                RETURN_FALSE;
            }
        } else if ( c == 'i' ) {
            int64_t val_i64 = 0;
            if ( ufr_get(link, "i", &val_i64) > 0 ) {
                add_next_index_long(return_value, val_i64);
                count += 1;
            }
        } else if ( c == 's' ) {
            char val_str[1024];
            if ( ufr_get(link, "s", val_str) > 0 ) {
                add_next_index_string(return_value, val_str);
                count += 1;
            }
        } else if ( c == 'b' ) {
            const size_t val_size = ufr_get_size(link);
            const char* val_str = ufr_get_raw_ptr(link);
            add_next_index_stringl(return_value, val_str, val_size);
            count += 1;
        } else if ( c == 'a' ) {
            zval array;
            array_init(&array);
            // ufr_enter_array(&link);
            // const size_t val_size = ufr_get_size(link);
            // const char* val_str = ufr_get_raw_ptr(link);
            add_next_index_zval(return_value, &array);
            // zval_ptr_dtor(&array);
            count += 1;
        }
    }

    //
    if ( count == 0 ) {
        RETURN_FALSE;
    }

    // Success: return the array "return_value"
}
