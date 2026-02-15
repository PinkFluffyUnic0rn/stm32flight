library(pracma)
library(geigen)
library(conicfit)

range = 500;

printf <- function(...) invisible(print(sprintf(...)))

args = commandArgs(trailingOnly=TRUE)

png("file.png", width = 10000, height = 5000);
t = read.table(args[1], header = FALSE, sep = " ");

start = 0
end = nrow(t)

magx = t[start:end, 2];
magy = t[start:end, 3];
magz = t[start:end, 4];

xz = matrix(nrow = 0, ncol=2);
for (i in 1:nrow(t)) {
	if (magy[i] < -range || magy[i] > range)
		next;

	xz = rbind(xz, c(magx[i], magz[i]));
}

eltaub <- EllipseFitByTaubin(xz);
elxz <- AtoG(eltaub)$ParG
xzc5<-calculateEllipse(elxz[1], elxz[2], elxz[3], elxz[4], 180 / pi * elxz[5]);


yz = matrix(nrow = 0, ncol=2);
for (i in 1:nrow(t)) {
	if (magx[i] < -range || magx[i] > range)
		next;

	yz = rbind(yz, c(magy[i], magz[i]));
}

eltaub <- EllipseFitByTaubin(yz);
elyz <- AtoG(eltaub)$ParG
yzc5<-calculateEllipse(elyz[1], elyz[2], elyz[3], elyz[4], 180 / pi * elyz[5]);

m = min(elxz[3], elyz[3], elxz[4]);

printf("xoffset: %f", -elxz[1]);
printf("yoffset: %f", -elyz[1]);
printf("zoffset: %f", -elxz[2]);
printf("xscale: %f", m / elxz[3]);
printf("yscale: %f", m / elyz[3]);
printf("zscale: %f", m / elxz[4]);

par(mfrow = c(1, 2))

plot(xz[,1], xz[,2], col='blue', type='p', lwd = 30);
lines(xzc5[,1],xzc5[,2],col='red', type='l', lwd = 30);

plot(yz[,1], yz[,2], col='blue', type='p', lwd = 30);
lines(yzc5[,1],yzc5[,2],col='red', type='l', lwd = 30);
