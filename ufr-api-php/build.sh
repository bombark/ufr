#!/bin/bash

phpize
./configure --enable-php-ufr
make 

# Install
# sudo make install

# Teste
# php -d extension=php_ufr.so -r 'ufr_publishera();'
# php -d extension=php_ufr.so -r 'helloworld_php();'