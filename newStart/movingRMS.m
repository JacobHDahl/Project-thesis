function RMSvalue = movingRMS(data)
N = length(data);
dims = size(data);
%Check if row vector
if dims(1) == 1
    sumSquared = data*data';
else
    sumSquared = data'*data;
end

RMSvalue = sqrt(sumSquared/N);
end




