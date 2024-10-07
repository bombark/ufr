phpize
./configure --enable-php-helloworld
make 
sudo make install

php -d extension=php_helloworld.so -r 'helloworld_php();'
