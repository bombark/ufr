phpize
./configure --enable-php-ufr
make 
sudo make install

php -d extension=php_ufr.so -r 'ufr_publishera();'
# php -d extension=php_ufr.so -r 'helloworld_php();'