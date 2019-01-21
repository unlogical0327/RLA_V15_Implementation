% 测试数据
x = 1:50;
y = x + rand(1,50)*10;

% 设置高斯模板大小和标准差
r        = 3;
sigma    = 1;
size(y)
y_filted = Gaussianfilter(r, sigma, y);

% 作图对比
figure
plot(x, y, x, y_filted);
title('高斯滤波');
legend('滤波前','滤波后','Location','northwest')
