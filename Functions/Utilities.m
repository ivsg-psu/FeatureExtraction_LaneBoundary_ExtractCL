classdef Utilities < handle
    
    properties
        
        
    end
    
    methods
        
        function [data, outlier_ranges] = removeOutliers(obj,data, max_standard_deviation_sigma)
            
            data_mean = mean(data,'omitnan');
            data_std = std(data,'omitnan');

            data_outliers = find(abs(abs(data)-abs(data_mean)) > max_standard_deviation_sigma * data_std | isnan(data));

            outlier_ranges = [];

            if ~isempty(data_outliers)

                outlier_edge_indices = find(diff(data_outliers) ~= 1) + 1;

                if isempty(outlier_edge_indices)
                    outlier_edge_indices = 1;
                    number_of_outlier_regions = 1;
                else
                    number_of_outlier_regions = length(outlier_edge_indices) + 1;
                end

                outlier_ranges = zeros(number_of_outlier_regions,2);

                for i = 1:number_of_outlier_regions

                    if i == 1

                        if number_of_outlier_regions == 1

                            outlier_ranges(i,:) = [data_outliers(1) data_outliers(end)];

                        else

                            outlier_ranges(i,:) = [data_outliers(1) data_outliers(outlier_edge_indices(1)-1)];

                        end

                    elseif i < number_of_outlier_regions

                        outlier_ranges(i,:) = [data_outliers(outlier_edge_indices(i-1)) data_outliers(outlier_edge_indices(i)-1)];

                    else

                        outlier_ranges(i,:) = [data_outliers(outlier_edge_indices(i-1)) data_outliers(end)];

                    end

                end
                
                %%% Replace the outliers with the average value on the values before and after the region
                data = removeRangeOfData(obj,data,outlier_ranges);
                
            end
            
        end
        
        function data = removeRangeOfData(obj,data,ranges)

            if ~isempty(ranges)

                for i = 1:size(ranges,1)

                    left_previous_ind = ranges(i,1)-1;
                    right_after_ind = ranges(i,2)+1;
                    
                    if left_previous_ind < 1
                        left_previous_ind = 1;
                    end

                    if right_after_ind > length(data)
                        right_after_ind = length(data);
                    end

                    if ranges(i,2) ~= length(data)
                        
                        if left_previous_ind == 1

                            data(left_previous_ind:right_after_ind-1) = data(right_after_ind);
                            
                        else
                            
                            data(left_previous_ind+1:right_after_ind-1) = round(mean(data([left_previous_ind right_after_ind])));
                            
                        end

                    else
                        
                        data(ranges(i,1):ranges(i,2)) = data(ranges(i,1)-1);
                        
                    end

                end

            end
            
        end
        
        function m = leastSquares(obj, X,Y)

            if rcond(X' * X) > 100 * eps

                m = X' * X \ X' * Y;

            else

                m = nan * ones(size(X,2),1);

            end

        end
        
        function [x, P] = recursiveLeastSquares(obj, x, y, P, lambda)

            k = inv(lambda)*P*x / (1 + inv(lambda)*x*P*x);
            e = y - x;
            x = x + k*e;
            P = inv(lambda) * P - inv(lambda)*k*x*P;

        end
        
    end

end