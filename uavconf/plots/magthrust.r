printf <- function(...) invisible(print(sprintf(...)))

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
	plot(values, ylim = c(yfrom, yto), type = "l",
		xaxs="i", col = "blue", lwd = 5, axes=FALSE);
	legend("topright", legend = title, col = "blue", cex=15);
	axis(side = 1, lwd = 5, at=seq(0, (end - start), freq),
		labels=seq(0, (end - start) / freq, 1))
	axis(side = 2, lwd = 5, at=round(seq(yfrom, yto,
		(yto - yfrom) / 7), digits = 3),
		mgp = c(10, 10, 0), las=1)
}

args = commandArgs(trailingOnly=TRUE)

t = read.table(args[1], header = FALSE, sep = " ");

freq = 256

start = 0
end = 2 * nrow(t) / 3

magx = t[start:end, 2];
magy = t[start:end, 3];
magz = t[start:end, 4];
m1 = t[start:end, 6];
m2 = t[start:end, 7];
m3 = t[start:end, 8];
m4 = t[start:end, 9];

magx_p = lpf1t(magx, 0.025)
magy_p = lpf1t(magy, 0.025)
magz_p = lpf1t(magz, 0.025)

startpos = 3 * freq

thrust = 0;
for (i in 1:length(m1)) {
	thrust[i] = (m1[i] + m2[i] + m3[i] + m4[i]) / 4.0;
	if (thrust[i] < 0.0) thrust[i] = 0;
}

avgx = avgy = avgz = 0.0
for (i in 1:startpos) {
	avgx = avgx + magx_p[i];
	avgy = avgy + magy_p[i];
	avgz = avgz + magz_p[i];
}

avgx = avgx / (startpos);
avgy = avgy / (startpos);
avgz = avgz / (startpos);

diffx = diffy = diffz = 0.0
cnt = 0
for (i in 1:length(magx_p)) {
	if (thrust[i] < 0.0001)
		next

	cnt = cnt + 1

	diffx = diffx + (magx_p[i] - avgx) / thrust[i];
	diffy = diffy + (magy_p[i] - avgy) / thrust[i];
	diffz = diffz + (magz_p[i] - avgz) / thrust[i];
}

diffx = -diffx / cnt
diffy = -diffy / cnt
diffz = -diffz / cnt

printf("diff x: %f", diffx);
printf("diff y: %f", diffy);
printf("diff z: %f", diffz);

png("magorig.png", width = 10000, height = 10000);

par(mfrow = c(4, 1))
par(mar = c(20, 50, 0, 0), cex.axis=10, mgp = c(15, 15, 0), tck=-0.05)

logplot("x", magx_p, min(magx_p), max(magx_p), linepos)
logplot("y", magy_p, min(magy_p), max(magy_p), linepos)
logplot("z", magz_p, min(magz_p), max(magz_p), linepos)
logplot("thrust", thrust, min(thrust), max(thrust), linepos)

for (i in 1:length(magx_p)) {
	magx_p[i] = magx_p[i] + 1942.314 * (thrust[i]);
	magy_p[i] = magy_p[i] - 125.647 * (thrust[i]);
	magz_p[i] = magz_p[i] + 351.655 * (thrust[i]);
}

png("magcorrected.png", width = 10000, height = 10000);

par(mfrow = c(4, 1))
par(mar = c(20, 50, 0, 0), cex.axis=10, mgp = c(15, 15, 0), tck=-0.05)

logplot("x", magx_p, min(magx_p), max(magx_p), linepos)
logplot("y", magy_p, min(magy_p), max(magy_p), linepos)
logplot("z", magz_p, min(magz_p), max(magz_p), linepos)
logplot("thrust", thrust, min(thrust), max(thrust), linepos)
