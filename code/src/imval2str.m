function dig = imval2str(i)
%% This function converts the input number to a string of three characters. 
% For instance, an input of '1' is returned as '001'

if i<10
    dig=strcat('000',num2str(i));
elseif i<100
    dig=strcat('00',num2str(i));
elseif i<1000
    dig=strcat('0',num2str(i));
elseif i<10000
    dig=strcat(num2str(i));
else
    error('The data file too big to process further that is number >10000.');
end
end

