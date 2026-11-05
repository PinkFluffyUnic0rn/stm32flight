pkg load signal;
pkg load parallel;
pkg load image;

#odr = 2000;
#file = "~/pidtest.txt";

odr = 8000;

#file = "~/pidtestpitchroll1.txt";
#file = "~/gyroxyz.txt";
#file = "~/pidgyroxyzrpsth1.txt";
#file = "~/gyroxyznoprops1.txt";
file = "~/gyroxyzseparatemotors5.txt";
datacol = 2;
throttlecol = 5;

#throttlecol1 = 6;
#throttlecol2 = 7;
#throttlecol3 = 8;
#throttlecol4 = 9;


function [b, a] = buildnotch(frq, bw, odr)
  r = 1 - 3 * bw / 8000;

  w0 = 2 * pi * frq / odr;
  b1 = [1, -2 * cos(w0), 1];
  a1 = [1, -2 * r * cos(w0), r ^ 2];


  w0 = 2 * pi * frq / odr * 2;
  b2 = [1, -2 * cos(w0), 1];
  a2 = [1, -2 * r * cos(w0), r ^ 2];

  a = conv(a1, a2);
  b = conv(b1, b2);

endfunction

function [b, a] = buildbiquad(frq, bw, odr)

  r = 1 - 3 * bw / odr;

  w0 = 2 * pi * frq / odr;
  b = [1, -2 * cos(w0), 1];
  a = [1, -2 * r * cos(w0), r ^ 2];

endfunction

function [b, a] = buildbiquaddir(frq, bw, odr)
  w0 = 2 * pi * frq / odr;
  a = sin(w0) / (2 * bw);
  a0 = 1 + a;

  b = [1/a0, -2 * cos(w0) / a0, 1 / a0];
  a = [1, -2 * cos(w0) / a0, (1 - a) / a0];

endfunction

function [b, a] = buildpt1(frq, odr)
  v = 2 * pi * frq / odr;
  alpha = 1 - v / (v + 1);

  a = [1, -alpha];
  b = [0, (1 - alpha)];

endfunction

function [b, a] = buildfilter(frq, bw, cutoff, odr)
  [b1, a1] = buildnotch(frq, bw, odr);

  [b2, a2] = buildpt1(cutoff, odr);

  a = conv(a1, a2);
  a = conv(a, a2);
  a = conv(a, a2);

  b = conv(b1, b2);
  b = conv(b, b2);
  b = conv(b, b2);

endfunction

throttle = load(file)(:,throttlecol);

datae = load(file)(:,datacol);
datalen = length(datae);

#throttle1 = load(file)(:,throttlecol1);
#throttle2 = load(file)(:,throttlecol2);
#throttle3 = load(file)(:,throttlecol3);
#throttle4 = load(file)(:,throttlecol4);

bandwidth = 2;
cutoff = 500;

[b, a] = buildbiquaddir(250, bandwidth, odr);
sf1 = zeros(max(length(a), length(b)) -1 , 1);
sf2 = zeros(max(length(a), length(b)) -1 , 1);

for i = 1 : datalen
  [b, a] = buildbiquaddir(throttle(i), bandwidth, odr);
  [v, sf1] = filter(b, a, datae(i), sf1);

  [b, a] = buildbiquaddir(throttle(i) * 2, bandwidth, odr);
  [v, sf2] = filter(b, a, v, sf2);

  [b, a] = buildbiquaddir(throttle(i) * 3, bandwidth, odr);
  [v, sf3] = filter(b, a, v, sf2);

  [H, w] = freqz(a, b, 8000);

if (throttle(i) > 280)
  [b1, a1] = buildbiquaddir(throttle(i), bandwidth, odr);
  [b2, a2] = buildbiquaddir(throttle(i) * 2, bandwidth, odr);
  [b3, a3] = buildbiquaddir(throttle(i) * 3, bandwidth, odr);

  thr = throttle(i)

  a = conv(a1, a2);
  a = conv(a, a3);

  b = conv(b1, b2);
  b = conv(b, b3);

[H, w] = freqz(a, b, 8000);

figure;
plot(w / pi * odr /2, unwrap(angle(H)));
xlim([0,2000]);
grid on;
return;
endif


  dataf(i, :) = v;
endfor

[H, w] = freqz(a, b, 8000);

figure;
plot(w / pi * odr /2, unwrap(angle(H)));
xlim([0,2000]);
grid on;

return;

#{
sf1 = zeros(max(length(a), length(b)) -1 , 1);
sf2 = zeros(max(length(a), length(b)) -1 , 1);
sf3 = zeros(max(length(a), length(b)) -1 , 1);
sf4 = zeros(max(length(a), length(b)) -1 , 1);
sf5 = zeros(max(length(a), length(b)) -1 , 1);
sf6 = zeros(max(length(a), length(b)) -1 , 1);
sf7 = zeros(max(length(a), length(b)) -1 , 1);
sf8 = zeros(max(length(a), length(b)) -1 , 1);
sf9 = zeros(max(length(a), length(b)) -1 , 1);
sf10 = zeros(max(length(a), length(b)) -1 , 1);
sf11 = zeros(max(length(a), length(b)) -1 , 1);
sf12 = zeros(max(length(a), length(b)) -1 , 1);

for i = 1 : datalen
  v = datae(i);

  [b, a] = buildbiquad(throttle1(i), bandwidth, odr);
  [v, sf1] = filter(b, a, v, sf1);

  [b, a] = buildbiquad(throttle1(i) * 2, bandwidth, odr);
  [v, sf2] = filter(b, a, v, sf2);

  [b, a] = buildbiquad(throttle1(i) * 3, bandwidth, odr);
  [v, sf3] = filter(b, a, v, sf3);

  [b, a] = buildbiquad(throttle2(i), bandwidth, odr);
  [v, sf4] = filter(b, a, v, sf4);

  [b, a] = buildbiquad(throttle2(i) * 2, bandwidth, odr);
  [v, sf5] = filter(b, a, v, sf5);

  [b, a] = buildbiquad(throttle2(i) * 3, bandwidth, odr);
  [v, sf6] = filter(b, a, v, sf6);

  [b, a] = buildbiquad(throttle3(i), bandwidth, odr);
  [v, sf7] = filter(b, a, v, sf7);

  [b, a] = buildbiquad(throttle3(i) * 2, bandwidth, odr);
  [v, sf8] = filter(b, a, v, sf8);

  [b, a] = buildbiquad(throttle3(i) * 3, bandwidth, odr);
  [v, sf9] = filter(b, a, v, sf9);

  [b, a] = buildbiquad(throttle4(i), bandwidth, odr);
  [v, sf10] = filter(b, a, v, sf10);

  [b, a] = buildbiquad(throttle4(i) * 2, bandwidth, odr);
  [v, sf11] = filter(b, a, v, sf11);

  [b, a] = buildbiquad(throttle4(i) * 3, bandwidth, odr);
  [v, sf12] = filter(b, a, v, sf12);

  dataf(i, :) = v;
endfor
#}
freqmap = pararrayfun(32, @(n) furier(dataf, n, 1000, odr), 1:(datalen - odr), "UniformOutput", false);
freqmap = cell2mat(freqmap);

freqmap = imresize(freqmap, [1000 30000]);

#plot(throttle);

imagesc(freqmap);
caxis([0 100]);
#caxis([0 25]);
