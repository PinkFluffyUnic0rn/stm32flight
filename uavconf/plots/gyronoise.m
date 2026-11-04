pkg load signal;
pkg load parallel;
pkg load image;

function [b, a] = buildnotch(frq, bw)
  r = 1 - 3 * bw / 8000;

  w0 = 2 * pi * frq / 8000;
  b1 = [1, -2 * cos(w0), 1];
  a1 = [1, -2 * r * cos(w0), r ^ 2];

  w0 = 2 * pi * frq / 8000 * 2;
  b2 = [1, -2 * cos(w0), 1];
  a2 = [1, -2 * r * cos(w0), r ^ 2];

  a = conv(a1, a2);
  b = conv(b1, b2);

endfunction

throttle = load("~/pidgyroxyzrpsth1.txt")(:,5);
datae = load("~/pidgyroxyzrpsth1.txt")(:,2);

[b1, a1] = buildnotch(throttle(i), 300);
#[b2, a2] = butter(2, 600/4000, 'low');
#[b2, a2] = cheby1(2, 0.5, 600/4000, 'low');

dt = 1 / 8000;
v = 2 * pi * 600 / 8000;
alpha = 1 - v / (v + 1);

a2 = [1, -alpha];
b2 = [0, (1 - alpha)];

a = conv(a1, a2);
a = conv(a, a2);

b = conv(b1, b2);
b = conv(b, b2);

b = conv(b1, b2);
b = conv(b, b2);

#a = a2;
#b = b2;

sf = zeros(max(length(a), length(b)) -1 , 1);

for i = 1 : 230000
  [b1, a1] = buildnotch(throttle(i), 300);

#  [b2, a2] = butter(2, 300/4000, 'low');
#  [b2, a2] = cheby1(2, 0.5, 400/4000, 'low');

  a = conv(a1, a2);
  a = conv(a, a2);

  b = conv(b1, b2);
  b = conv(b, b2);

  b = conv(b1, b2);
  b = conv(b, b2);

#  a = a2;
#  b = b2;

 [dataf(i, :), sf] = filter(b, a, datae(i), sf);
endfor

freqmap = pararrayfun(32, @(n) furier(throttle, n), 1:222000, "UniformOutput", false);

#freqmap = pararrayfun(32, @(n) furier(datae, n), 1:222000, "UniformOutput", false);
freqmap = cell2mat(freqmap);

freqmap = imresize(freqmap, [1500 30000]);

#plot(throttle);

imagesc(freqmap);
caxis([0 300]);
