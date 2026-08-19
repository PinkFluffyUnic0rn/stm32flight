pkg load signal;
pkg load parallel;
pkg load image;

function [b, a] = buildnotch(frq, bw)
  r = 1 - 3 * bw / 4000;

  w0 = 2 * pi * frq / 4000;
  b1 = [1, -2 * cos(w0), 1];
  a1 = [1, -2 * r * cos(w0), r ^ 2];

  w0 = 2 * pi * frq / 4000 / 2;
  b2 = [1, -2 * cos(w0), 1];
  a2 = [1, -2 * r * cos(w0), r ^ 2];

  a = conv(a1, a2);
  b = conv(b1, b2);

endfunction


#data = load("accvals.txt")(:,2);
#data = load("stm32flight/uavconf/plots/data/gyronoise2.txt")(:,4);
data = load("~/gyronoise10.txt")(:,2);

#throttle = load("stm32flight/uavconf/plots/data/gyronoise2.txt")(:,5);

datae = load("~/filtertest/123.txt")(:,2);

[b, a] = buildnotch(640, 300);

sf = zeros(max(length(a), length(b)) -1 , 1);

#for i = 1 : 80000
#  [b, a] = buildnotch(500 + ((throttle(i)) ^ 2) * (640 - 500) * 1.23456, 300);
#  [b, a] = buildnotch(620, 300);

#  [dataf(i, :), sf] = filter(b, a, data(i), sf);
#endfor

freqmap = pararrayfun(32, @(n) furier(datae, n), 1:76000, "UniformOutput", false);
freqmap = cell2mat(freqmap);

freqmap = imresize(freqmap, [1000 30000]);

#for i = 1 : 80000
#  throttlesc(i) = 500 + ((throttle(i) / 0.9) ^ 2) * (640 - 500);
#endfor

#plot(throttlesc);

imagesc(freqmap);
caxis([0 300]);
