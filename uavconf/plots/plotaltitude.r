png("file.png", width = 10000, height = 10000);
t = read.table("alt5.txt", header = FALSE, sep = " ");

freq = 64

start = 0
end = nrow(t)

#start = 83 * freq;
#end = 91 * freq;

linepos = 2.0 * freq;

baralt = t[start:end, 3];
alt = t[start:end, 4];
climbrate = t[start:end, 2];
ch0 = t[start:end, 5];
va = t[start:end, 6];
altdif = t[start:end, 7];
climbcor = t[start:end, 8];
thrustcor = t[start:end, 9];

alpha = exp(-1.0 / freq / 0.25)

va_p = 0;
s = 0;
for (i in 1:length(climbrate)) {
	s = alpha * s + (1 - alpha) * va[i];
	va_p[i] = s;
}


logplot <- function(title, values, yfrom, yto, line) {
	plot(values, ylim = c(yfrom, yto), type = "l", xaxs="i", col = "blue", lwd = 5, axes=FALSE);
	abline(v = line, col = "red", lwd = 5)
	legend("topright", legend = title, col = "blue", cex=15);
	axis(side = 1, lwd = 5, at=seq(0, (end - start), freq), labels=seq(0, (end - start) / freq, 1))
	axis(side = 2, lwd = 5, at=round(seq(yfrom, yto, (yto - yfrom) / 7), digits = 3), mgp = c(10, 10, 0), las=1)
}

par(mfrow = c(8, 1))
par(mar = c(20, 50, 0, 0), cex.axis=10, mgp = c(15, 15, 0), tck=-0.05)

logplot("barometric altitude", baralt, min(baralt), max(baralt), linepos)
logplot("calculated altitude", alt, min(baralt), max(baralt), linepos)
logplot("climb speed", climbrate, min(climbrate), max(climbrate), linepos)
logplot("pitch control", ch0, -1.0, 1.0, linepos)
logplot("climb acceleration", va_p, 0.8, 1.2, linepos)
logplot("altitude difference", altdif, -2.0, 2.0, linepos)
logplot("climb speed correction", climbcor, -0.3, 0.3, linepos)
logplot("climb acceleration correction", thrustcor, 0.756 - 0.1, 0.756 + 0.1, linepos)
