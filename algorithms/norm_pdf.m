function [pdf] = norm_pdf(x, mu, sigma)
%NORM_PDF Summary of this function goes here
pdf = (exp(-0.5*((x - mu)/sigma).^2))/(sigma*sqrt(2*pi));

end

