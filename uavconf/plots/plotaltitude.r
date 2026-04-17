args = commandArgs(trailingOnly=TRUE);


png("file.png", width = 10000, height = 10000);
t = read.table(args[1], header = FALSE, sep = " ");

freq = 1000

#start = 0
#end = nrow(t)

start = 14 * freq
end = 23 * freq


linepos = 12.7 * freq;

va = t[start:end, 2];
avgthr = t[start:end, 3];
baralt = t[start:end, 4];
climbrate = t[start:end, 5];
alt = t[start:end, 6];
ch0 = t[start:end, 7];
ch1 = t[start:end, 8];
ch2 = t[start:end, 9];

alpha = exp(-1.0 / freq / 0.25)

va_p = va[1];
s = va[1];
for (i in 1:length(va)) {
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

maxvel = 20.0;

vel = 0
for (i in 1:length(va))
	vel[i] = 0.0;

accs = as.integer(3.5 * freq);
acce = as.integer(5.5 * freq);

for (i in accs:acce)
	vel[i] = maxvel * (i - accs) / (acce - accs);


accs = as.integer(5.5 * freq);
acce = as.integer(6.5 * freq);

for (i in accs:acce)
	vel[i] = maxvel;


accs = as.integer(6.5 * freq);
acce = as.integer(7.0 * freq);

for (i in accs:acce)
	vel[i] = maxvel - maxvel * (i - accs) / (acce - accs);



thrder = 0.0;
thrder[0] = avgthr[0];
a = 0.05 / (0.05 + 1/freq)
for (i in 2:length(avgthr)) {
	thrder[i] = a * (thrder[i - 1] + avgthr[i] - avgthr[i - 1])
}

altthr = 0;

thrust = 0.0
for (i in 1:length(avgthr)) {
	thrust[i] = avgthr[i] ^ 2;
}

for (i in 1:length(baralt)) {
#	altcorr = 5.2 * thrust[i] - 3.7 * thrust[i]
	#altcorr = 8 * (avgthr[i] ^ 2.5) - 4.7
#	if (altcorr < 0)
#		altcorr = 0.0

	altcorr = 0.004 * vel[i] * vel[i];

#	altcorr = 0.0057 * vel[i] ^ 2.5;

	altthr[i] = baralt[i] - altcorr; 
}

#par(mfrow = c(8, 1))
par(mfrow = c(7, 1))
par(mar = c(20, 50, 0, 0), cex.axis=10, mgp = c(15, 15, 0), tck=-0.05)

logplot("climb acceleration", va_p, min(va_p), max(va_p), linepos)
logplot("average throttle", avgthr, 0.28, max(avgthr), linepos)
#logplot("thrust", thrust, 0.4, 1.0, linepos)
#logplot("average throttle derivitive", thrder, min(thrder), max(thrder), linepos)
#logplot("climb speed", climbrate, min(climbrate), max(climbrate), linepos)
logplot("modeled velocity", vel, min(vel), max(vel), linepos)
logplot("barometric altitude", baralt, min(baralt), max(baralt), linepos)
logplot("thrust corrected altitude", altthr, -59, -57, linepos)
logplot("calculated altitude", alt, min(alt), max(alt), linepos)
logplot("channel 1", ch1, min(ch1), max(ch1), linepos)
#logplot("channel 2", ch2, min(ch2), max(ch2), linepos)
