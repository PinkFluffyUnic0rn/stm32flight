library(pracma)
library(geigen)
library(conicfit)

range = 300;
xcenter = -1000;
ycenter = 200;

printf <- function(...) invisible(print(sprintf(...)))

args = commandArgs(trailingOnly=TRUE)

#png("circle.png", width = 10000, height = 5000);
t = read.table(args[1], header = FALSE, sep = " ");

start = 0
end = nrow(t)

gyrox = t[start:end, 2];
gyroy = t[start:end, 3];
gyroz = t[start:end, 4];

printf("xoffset: %f", -mean(gyrox));
printf("yoffset: %f", -mean(gyroy));
printf("zoffset: %f", -mean(gyroz));
