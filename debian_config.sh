./configure \
	--enable-version3 \
	--enable-libdrm --enable-librga --enable-rkmpp \
	--enable-ffplay \
	--disable-v4l2_m2m \
	--disable-decoder=h264_v4l2m2m \
	--disable-decoder=vp8_v4l2m2m \
	--disable-decoder=mpeg2_v4l2m2m \
	--disable-decoder=mpeg4_v4l2m2m \
	--disable-static --enable-shared
