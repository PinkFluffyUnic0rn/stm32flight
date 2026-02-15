#start = 1728;
#end = 2048;

start = 3800;
end = 4000;

png("file.png", width = 4000, height = 2000);

t = read.table("123.log", header = FALSE, sep = " ");
#t = read.table("logyaw2.txt", header = FALSE, sep = " ");
gz = t[start:end, 7];

lt = t[start:end, 21];
lb = t[start:end, 22];
rb = t[start:end, 23];
rt = t[start:end, 24];


tmp = 2 * 3.14159 * 40 / 4000;
alpha = 1 - tmp / (tmp + 1);

s = 0;
vpt1 = c();
for (i in 1:length(gz)) {
	s = alpha * s + (1 - alpha) * gz[i];
	vpt1[i] = s;
}

fr = c();
for (i in 1:256) {
	fr[i] = Mod(fft(gz[512:768])[i]);
}


frpt1 = c();
for (i in 1:256) {
	frpt1[i] = Mod(fft(vpt1[512:768])[i]);
}



lbpt1 = c();
rbpt1 = c();
ltpt1 = c();
rtpt1 = c();

tmp = 2 * 3.14159 * 40 / 4000;
alpha = 1 - tmp / (tmp + 1);

s = 0;
for (i in 1:length(gz)) {
	s = alpha * s + (1 - alpha) * lb[i];
	lbpt1[i] = s;
}

s = 0;
for (i in 1:length(gz)) {
	s = alpha * s + (1 - alpha) * rb[i];
	rbpt1[i] = s;
}

s = 0;
for (i in 1:length(gz)) {
	s = alpha * s + (1 - alpha) * rt[i];
	rtpt1[i] = s;
}

s = 0;
for (i in 1:length(gz)) {
	s = alpha * s + (1 - alpha) * lt[i];
	ltpt1[i] = s;
}

par(mfrow = c(2, 1))

plot(vpt1, type = "l", ylim = c(-10, 10));
grid(nx = length(gz) / 64, ny = 20, lty = 2, col = "gray", lwd = 2);
lines(gz, type = "l", ylim = c(-10, 10), col = "orange");


plot(lt, type = "l", ylim = c(0.37, 0.47), col = "orange");
lines(ltpt1, type = "l", ylim = c(0.37, 0.47), col = "red");
grid(nx = length(gz) / 64, ny = 10, lty = 2, col = "gray", lwd = 2);

lines(lb, type = "l", ylim = c(0.37, 0.47), col = "purple");
lines(lbpt1, type = "l", ylim = c(0.37, 0.47), col = "blue");

lines(rb, type = "l", ylim = c(0.37, 0.47), col = "orange");
lines(rbpt1, type = "l", ylim = c(0.37, 0.47), col = "red");

lines(rt, type = "l", ylim = c(0.37, 0.47), col = "purple");
lines(rtpt1, type = "l", ylim = c(0.37, 0.47), col = "blue");

