start = 0;
end = 41584;

png("file.png", width = 10000, height = 5000);

#t = read.table("ztestzx.txt", header = FALSE, sep = " ");
#t = read.table("ztest3.txt", header = FALSE, sep = " ");
#t = read.table("ztest-zx.txt", header = FALSE, sep = " ");
#t = read.table("ztestzx1.txt", header = FALSE, sep = " ");
t = read.table("ztestzx2.txt", header = FALSE, sep = " ");
accz = t[start:end, 2];
temp = t[start:end, 3];
alt = t[start:end, 4];
accx = t[start:end, 5];

#tmp = 2 * 3.14159 * 0.125 / 64;
#alpha = 1 - tmp / (tmp + 1);

alpha = exp(-1.0 / 64 / 2)

accz_p = 0;
s = 0;
for (i in 1:length(accz)) {
	s = alpha * s + (1 - alpha) * accz[i];
	accz_p[i] = s;#// + (temp[i] - 25) * 0.00058;
}

accx_p = 0;
s = 0;
for (i in 1:length(accx)) {
	s = alpha * s + (1 - alpha) * accx[i];
	accx_p[i] = s;
}

logplot <- function(values, yfrom, yto) {
	plot(values, ylim = c(yfrom, yto), type = "l", xaxs="i", col = "blue", lwd = 5, axes=FALSE);
	axis(side = 1, lwd = 5, at=seq(0, (end - start), 3840), labels=seq(0, 10, 1))
	axis(side = 2, lwd = 5, at=round(seq(yfrom, yto, (yto - yfrom) / 7), digits = 3), mgp = c(10, 10, 0), las=1)
}

par(mfrow = c(4, 1))
#par(mfrow = c(4, 1))
par(mar = c(20, 50, 0, 0), cex.axis=10, mgp = c(15, 15, 0), tck=-0.05)

#logplot(temp, 34, 41)
#logplot(alt, 124.0, 126.0)
#logplot(accz_p, 1.003, 1.004)
#logplot(accx_p, 0.0405, 0.0415)

#logplot(alt, 127.3, 129.5)
#logplot(accz_p, 0.995, 1.0)
#logplot(accx_p, 0.088, 0.093)

#logplot(temp, 38, 43)
#logplot(alt, 65.0, 67.0)
#logplot(accz_p, -1.0025, -1.0035)
#logplot(accx_p, -0.022, -0.026)

#logplot(temp, 31, 40)
#logplot(alt, 144.0, 146.0)
#logplot(accz_p, 1.0073, 1.0083)
#logplot(accx_p, 0.013, 0.017)

logplot(temp, 16, 27)
logplot(alt, 77.0, 82.0)
logplot(accz_p, 1.0025, 1.0045)
logplot(accx_p, 0.013, 0.017)
