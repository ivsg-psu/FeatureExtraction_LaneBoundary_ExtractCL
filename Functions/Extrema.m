% This is a class of functions used to calculate the extrema in data, based on Pramod
% Vemulapalli's thesis work on optimal extrema extraction. It consists of
% calculations to determine the optimal filter, then application of that
% optimal filter.

classdef Extrema < handle
    
    properties
        flag_show_plots = 0;  % Whether or not to show plots
    end
    
    methods
        
        % The following function calculates the optimal filter to use to
        % extract extrema
        function filter = fcn_createOptimalExtremaFilter(~,data,input_filter_length)
            
            % The actual filter length has to include endpoints
            filter_length = input_filter_length + 2;
            data_length = length(data);

            % Initialize the X data vector
            X_data = zeros(filter_length-2);
            
            % Loop through the possible filter lengths, to construct an
            % information matrix, X_data. Usually the loop just passes
            % through once because data_length is far bigger than filter
            % length
            for filter_k = 0:(data_length - filter_length + 1):(filter_length - 1)

                % Find out how many rows of data we will need to keep
                number_of_rows = length(filter_k + 1 : (filter_k + data_length - filter_length + 1));

                % Create a placeholder array, Z
                Z = zeros(filter_length, number_of_rows);
                
                % For each of the filters, save a portion of the data into
                % a row of Z to construct the Z matrix                
                for row_i = 1:filter_length
                    Z(row_i,:) = data( filter_k + row_i : filter_k + data_length - filter_length + row_i);
                end

                % Create subset matrices y and z from matrix Z
                y = Z(2:filter_length-1,:);
                z = Z(1:filter_length-2,:);

                % Define Z1 and Z2, these are rows of data
                Z1 = Z(filter_length,:);                
                Z2 = Z(filter_length-1,:);
                
                % Calculate differences in the rows
                for row_i = 1:filter_length-2
                    y(row_i,:) = y(row_i,:) - Z1;
                    z(row_i,:) = z(row_i,:) - Z2;
                end

                % Fill in the X_data matrix, which is sum of information in
                % y and z
                X_data = X_data + y * y';
                X_data = X_data + z * z';

            end

            % J is an identidy matrix the size of our original filter
            J = eye(filter_length - 2);

            % Equation 44
            A = ones(filter_length - 2) + J; % Equation 44, this is the I + J_(2N+1)

            % Paragraph after Equation 44 in the thesis
            alpha = A \ X_data;

            % Calculate eigenvalues and eigenvectors
            [alpha_eigenvalues, alpha_eigenvectors] = eig(alpha);
            
            % Normalize the eigenvalues
            alpha_normalized1 = zeros(size(alpha_eigenvalues,1)+2, size(alpha_eigenvalues,2));
            alpha_normalized1(2:filter_length-1,:) = alpha_eigenvalues;
            alpha_normalized1(filter_length,:) = -sum(alpha_eigenvalues); % This sums the columns of eigenvalues and tucks the sum into the end (is this right?)
            alpha_normalized1 = alpha_normalized1 ./ sqrt(sum(alpha_normalized1.^2));  % This divides all eigenvalues by their sums, converting to 0 to 1 values

            % Now sort the eigenvectors
            [~,indices] = sort(diag(alpha_eigenvectors),'descend');
            
            % Choose the maximum eigenvalue (TODO: Is this right?)
            a = cumsum(alpha_normalized1(:,indices(1)));            
            
            % The filter is the cumulative sum
            filter = a(2:end-1,1); 
            1;
        end
        
        function [extrema, correlation] = fcn_findExtrema(obj,data,filter,number_of_extrema)
            % Use padarray to fill the array
            % pad_len = floor(length(filter)/2);
            % data_pad = padarray(data, [pad_len 0], 'symmetric');
            filter_normalized = filter/sum(filter);
            % Perform filtering by convolutiong filter with the data
            correlation = conv(data,filter_normalized,'same');

%             % Finding the peaks and valleys in one shot is slightly faster and we then
%             % do not have to sort.
%             find_total = find((correlation(2:end-1) > correlation(1:end-2) & correlation(2:end-1) > correlation(3:end)) | (correlation(2:end-1) < correlation(1:end-2) & correlation(2:end-1) < correlation(3:end))) + 1;
% 
%             % find_total = sort([peaks valleys]);
%             find_total_real = find_total(2:end-1); % I believe this is the correct one, not below
% %             find_total_real = find_total;
%             sum_total = abs(correlation(find_total(2:end-1)) - correlation(find_total(1:end-2))) + abs(correlation(find_total(2:end-1)) - correlation(find_total(3:end)));
% %             sum_total = abs(diff(correlation(find_total(1:end-1)))) + abs(diff(correlation(find_total(2:end))));
% 
%             [~, sort_indices] = sort(sum_total,'descend');
% %             [~, sort_indices] = sort(correlation(find_total_real),'descend');
% 
%             extrema = zeros(size(correlation));
% 
%             if length(sort_indices) >= number_of_extrema
%                 extrema(find_total_real(sort_indices(1:number_of_extrema))) = 1;
%             elseif length(sort_indices) < number_of_extrema
%                 extrema(find_total_real(sort_indices)) = 1;
%             end
            
            % get the peaks - these are locations where the correlation
            % output is higher than the values before, and values after
            peaks=find(correlation(2:end-1)>correlation(1:end-2) & correlation(2:end-1)>correlation(3:end));
            peaks=peaks+1;            

            % get the valleys 
            valleys=find(correlation(2:end-1)<correlation(1:end-2) & correlation(2:end-1)<correlation(3:end));
            valleys=valleys+1;

            % populate the peak data matrix 
            extrema=zeros(size(correlation));
            extrema(peaks)=1;
            extrema(valleys)=1;

            % Plot the results thus far?
            if obj.flag_show_plots
                figure(2020);                
                clf; hold on; grid minor;
                indices_data = 1:length(correlation);
                plot(indices_data,correlation);
                plot(peaks, correlation(peaks),'go');
                plot(valleys, correlation(valleys),'ro');
                plot(indices_data(extrema==1),correlation(extrema==1),'kx');
                title('Extrema');
            end
            
            % choose the right peaks
%             final_peaks = floor(0.03*length(correlation));
            final_peaks = number_of_extrema;
            find_total = find(extrema);  % Grab indices that have extrema
            % find_total_real = find_total(2:end-1);  % Drop off the endpoints
            
            % Take the sum of all the correlations - not sure what this
            % does, but it looks like it sums the CHANGE in correlations
            % between peaks to determine strength of the peaks.
            % sum_total = abs(correlation(find_total(2:end-1))-correlation(find_total(1:end-2)))+ abs(correlation(find_total(2:end-1))-correlation(find_total(3:end)));
            
            % Sort the peaks
            extrema_correlation = correlation(find_total);
            [~,sort_locs] = sort(extrema_correlation,'descend');
            
            % Initialize the extrema matrix
            extrema=zeros(size(correlation));

            % Save the extrema locations with an index of 1
            if length(sort_locs) > final_peaks
                extrema(find_total(sort_locs(1:final_peaks)))=1;
            else
                extrema(find_total(sort_locs))=1;
            end
        end
        
    end
    
end