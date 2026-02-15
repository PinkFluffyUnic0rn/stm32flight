png("file.png", width = 10000, height = 10000);
t = read.table("data/mag7.txt", header = FALSE, sep = " ");

freq = 128

start = 0
end = nrow(t)

linepos = 10.3 * freq;

magx = t[start:end, 2];
magy = t[start:end, 3];
magz = t[start:end, 4];
m1 = t[start:end, 6];
m2 = t[start:end, 7];
m3 = t[start:end, 8];
m4 = t[start:end, 9];


#ch2 = t[start:end, 5];
#ch3 = t[start:end, 6];
#yaw = t[start:end, 7];
#ch0 = t[start:end, 8];
#ch1 = t[start:end, 9];

lpf1t <- function(data, t) {
	r = 0;
	s = 0;

	alpha = exp(-1.0 / freq / t)
	
	for (i in 1:length(data)) {
		s = alpha * s + (1 - alpha) * data[i];
		r[i] = s;
	}

	return(r);
}

logplot <- function(title, values, yfrom, yto, line) {
	plot(values, ylim = c(yfrom, yto), type = "l", xaxs="i", col = "blue", lwd = 5, axes=FALSE);
	abline(v = line, col = "red", lwd = 5)
	legend("topright", legend = title, col = "blue", cex=15);
	axis(side = 1, lwd = 5, at=seq(0, (end - start), freq), labels=seq(0, (end - start) / freq, 1))
	axis(side = 2, lwd = 5, at=round(seq(yfrom, yto, (yto - yfrom) / 7), digits = 3), mgp = c(10, 10, 0), las=1)
}

magx_p = lpf1t(magx, 0.025)
magy_p = lpf1t(magy, 0.025)
magz_p = lpf1t(magz, 0.025)

thrust = 0;
for (i in 1:length(m1)) {
	thrust[i] = (m1[i] + m2[i] + m3[i] + m4[i]) / 4.0;
	if (thrust[i] < 0.0) thrust[i] = 0;
}

for (i in 1:length(magx_p)) {
	magx_p[i] = magx_p[i];# + 1559 * (thrust[i]);
}

heading = 0;
for (i in 1:length(magx_p)) {
	x = magx_p[i];
	y = (magy_p[i] + 75) * 1.0;
	heading[i] = atan2(y, x);# + 0.3264;


}

print(magx_p[1024]);
print(magx_p[1318]);
print(thrust[1318]);

par(mfrow = c(9, 1))
par(mar = c(20, 50, 0, 0), cex.axis=10, mgp = c(15, 15, 0), tck=-0.05)

logplot("x", magx_p, -2000, 500, linepos)
logplot("y", magy_p, min(magy_p), max(magy_p), linepos)
logplot("z", magz_p, min(magz), max(magz), linepos)
logplot("thrust", thrust, min(thrust), max(thrust), linepos)
logplot("heading", heading, -3.15, 3.15, linepos)
logplot("m1", m1, 0.0, 1.0, linepos)
logplot("m2", m2, 0.0, 1.0, linepos)
logplot("m3", m3, 0.0, 1.0, linepos)
logplot("m4", m4, 0.0, 1.0, linepos)
