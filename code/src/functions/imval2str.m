function dig = imval2str(style, i)
%%This function converts the input number to a string of style similar
% to the way image names have been stored in the dataset.
% For instance, image name has a style '0000000000' and input i is '1' then
% the function returns '0000000001'
len = length(style);
i = num2str(i);
dig = [style(1:(len-length(i))), i];
end