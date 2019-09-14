all:
	make clean
	mkdir build && cd build && cmake -DBUILD_EXAMPLES=ON -DWITH_DEBUG=ON -DWITH_GTEST=ON -DBUILD_DOC=ON -DWITH_VISUALIZATION=ON -DUSE_PYENV=ON ../ && make -j`nproc`

clean:
	rm -rf ./build*
