include $(PETALINUX)/software/petalinux-dist/tools/user-commons.mk

mydrv_test: mydrv_test.c
	$(CC) $(CFLAGS) $(LDFLAGS) -lpthread mydrv_test.c -o mydrv_test
zdmai_test: zdmai_test.c
	$(CC) $(CFLAGS) $(LDFLAGS) -lpthread zdmai_test.c -o zdmai_test
zgpio_test: zgpio_test.c
	$(CC) $(CFLAGS) $(LDFLAGS) -lpthread zgpio_test.c -o zgpio_test
spiterm: spiterm.c
	$(CC) $(CFLAGS) $(LDFLAGS) -lpthread spiterm.c -lrt -o spiterm
spidev_test: spidev_test.c
	$(CC) $(CFLAGS) $(LDFLAGS) spidev_test.c -o spidev_test	
clean:
	rm mydrv_test
	rm zgpio_test
	rm spiterm
	rm spidev_test
