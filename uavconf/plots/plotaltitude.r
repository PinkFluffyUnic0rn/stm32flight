args = commandArgs(trailingOnly=TRUE);


png("file.png", width = 10000, height = 10000);
t = read.table(args[1], header = FALSE, sep = " ");

freq = 1000

start = 0
end = nrow(t)

start = 13.0 * freq
end = 15.0 * freq

linepos = 12.7 * freq;

fa = t[start:end, 2];
va = t[start:end, 3];
baralt = t[start:end, 4];
climbrate = t[start:end, 5];
alt = t[start:end, 6];
ch0 = t[start:end, 7];
ch1 = t[start:end, 8];
ch2 = t[start:end, 9];

alpha = exp(-1.0 / freq / 0.25)

va_p = va[1];
s = va[1];
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

#par(mfrow = c(8, 1))
par(mfrow = c(7, 1))
par(mar = c(20, 50, 0, 0), cex.axis=10, mgp = c(15, 15, 0), tck=-0.05)

logplot("forward acceleration", fa, -1, 1, linepos)
logplot("climb acceleration", va_p, 0.9, 1.1, linepos)
logplot("climb speed", climbrate, min(climbrate), max(climbrate), linepos)
#logplot("barometric altitude", baralt, min(baralt), max(baralt), linepos)
logplot("barometric altitude", baralt, 11.2, 13.9, linepos)
logplot("calculated altitude", alt, 11.2, 13.9, linepos)
#logplot("channel 0", ch0, min(ch0), max(ch0), linepos)
logplot("channel 1", ch1, min(ch1), max(ch1), linepos)
logplot("channel 2", ch2, min(ch2), max(ch2), linepos)
