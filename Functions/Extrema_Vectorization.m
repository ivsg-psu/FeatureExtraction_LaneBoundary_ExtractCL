% This is a class of functions used to calculate the extrema in data, based on Pramod
% Vemulapalli's thesis work on optimal extrema extraction. It consists of
% calculations to determine the optimal filter, then application of that
% optimal filter.

classdef Extrema_Vectorization < handle
    
    properties
        flag_show_plots = 0;  % Whether or not to show plots
    end
    
    methods
        
        % The following function calculates the optimal filter to use to
        % extract extrema
        function filter = fcn_createOptimalExtremaFilter(~,data,input_filter_length)
            
            % Compute Optimal Extrema Filter for All Scans in One LiDAR Ring
            %
            % Input:
            %   data: [num_points x num_scans] matrix, each column represents one LiDAR scan intensity profile
            %   input_filter_length: The desired filter length (excluding endpoints)
            %
            % Output:
            %   filter: [input_filter_length x num_scans] matrix, storing optimal extrema filters for all scans

            % Get data dimensions

            [num_points, num_scans] = size(data);

            % Compute the actual filter length (including endpoints)
            filter_length = input_filter_length + 2;

            % Ensure filter length is valid
            if filter_length >= num_points
                error('Filter length is too large for the given number of points.');
            end

            % Initialize the X data vector
            X_data = zeros(filter_length-2,filter_length-2, num_scans);
            
            % Preallocate storage for filter results
            filter_all_scans = zeros(input_filter_length, num_scans);
   
            for filter_k = 0:(num_points - filter_length + 1):(filter_length - 1)
                
                % Find out how many rows of data we will need to keep
                num_valid_points = length(filter_k + 1 : (filter_k + num_points - filter_length + 1));
                Z = zeros(num_scans, num_valid_points, filter_length);
                for page_i = 1:filter_length
                    Z(:,:,page_i) = data(filter_k + page_i : filter_k + num_points - filter_length + page_i,:).';
                end

                Z_reshape = permute(Z, [3, 2, 1]);

                % Create subset matrices y and z from matrix Z
                y = Z_reshape(2:filter_length-1, :, :);
                z = Z_reshape(1:filter_length-2, :, :);

                % Define Z1 and Z2, these are rows of data
                Z1 = Z_reshape(filter_length,:,:);                
                Z2 = Z_reshape(filter_length-1,:,:);

                y_prime = y - Z1;
                z_prime = z - Z2;

                % Compute the information matrix X_data for all scans simultaneously
                X_data = pagemtimes(y_prime, 'none', permute(y_prime, [2, 1, 3]), 'none') + ...
                    pagemtimes(z_prime, 'none', permute(z_prime, [2, 1, 3]), 'none');
            end

            % Construct A matrix (same for all scans)
            J = eye(filter_length - 2);
            A = ones(filter_length - 2) + J;
            % 

            % Solve for optimal extrema filter for all scans at once (Fully Vectorized)
            Alpha = pagemldivide(A, X_data);  % Equivalent to solving A \ X_data for all scans at once

            % Compute eigenvalues and eigenvectors in a vectorized manner
            [alpha_eigenvectors, alpha_eigenvalues] = pageeig(Alpha);

            % Normalize eigenvalues
            alpha_normalized = zeros(size(alpha_eigenvectors,1)+2, size(alpha_eigenvectors,2), num_scans);
            alpha_normalized(2:filter_length-1, :, :) = alpha_eigenvectors;
            alpha_normalized(filter_length, :,:) = -sum(alpha_eigenvectors,1); % 
            alpha_normalized = alpha_normalized ./ sqrt(sum(alpha_normalized.^2));
            filter_all_scans = zeros(input_filter_length,num_scans);
            % Sort eigenvectors in descending order and extract the correct one
            for idx_scan = 1:num_scans
                alpha_eigenvalues_currentScan = alpha_eigenvalues(:,:,idx_scan);
                [~, sorted_indices] = sort(diag(alpha_eigenvalues_currentScan), 'descend');
                a = cumsum(alpha_normalized(:, sorted_indices(1),idx_scan), 1);
                filter_all_scans(:, idx_scan) = a(2:end-1,:);
            end
            filter = abs(filter_all_scans);

            % Store the extrema filter for all scans

        end
        
        function [extrema, correlation] = fcn_findExtrema(obj,data,filter,number_of_extrema)
    
            

            [num_points, num_scans] = size(data);

 

            extrema = zeros(num_points, num_scans);
            correlation = zeros(num_points,num_scans);
            % Perform filtering by convolutiong filter with the data
            for idx_scan = 1:num_scans
                data_currentScan = data(:,idx_scan);
                filter_currentScan = filter(:,idx_scan);
                
                filter_currentScan_normalized = filter_currentScan/sum(filter_currentScan);
                correlation_currentScan = conv(data_currentScan,filter_currentScan_normalized,'same');
            
                
                % data_currentScan_smooth = movmean(data_currentScan, filter_length, 'Endpoints', 'shrink');
                % correlation_currentScan(1:pad_len) = data_currentScan_smooth(1:pad_len);
                % correlation_currentScan(end-pad_len+1:end) = data_currentScan_smooth(end-pad_len+1:end);

                % correlation_currentScan = rescale(correlation_currentScan,0,1);
                % get the peaks - these are locations where the correlation
                % output is higher than the values before, and values after
                peaks=find(correlation_currentScan(2:end-1)>correlation_currentScan(1:end-2) & correlation_currentScan(2:end-1)>correlation_currentScan(3:end));
                peaks=peaks+1;

                % get the valleys
                valleys=find(correlation_currentScan(2:end-1)<correlation_currentScan(1:end-2) & correlation_currentScan(2:end-1)<correlation_currentScan(3:end));
                valleys=valleys+1;

                % populate the peak data matrix
                extrema_currentScan=zeros(size(correlation_currentScan));
                extrema_currentScan(peaks)=1;
                extrema_currentScan(valleys)=1;

                % Plot the results thus far?
                if obj.flag_show_plots
                    figure(2020);
                    clf; hold on; grid minor;
                    indices_data = 1:length(correlation_currentScan);
                    plot(indices_data,correlation_currentScan);
                    plot(peaks, correlation_currentScan(peaks),'go');
                    plot(valleys, correlation_currentScan(valleys),'ro');
                    plot(indices_data(extrema_currentScan==1),correlation_currentScan(extrema_currentScan==1),'kx');
                    title('Extrema');
                end

                % choose the right peaks
                %             final_peaks = floor(0.03*length(correlation));
                final_peaks = number_of_extrema;
                find_total = find(extrema_currentScan);  % Grab indices that have extrema
                % find_total_real = find_total(2:end-1);  % Drop off the endpoints

                % Take the sum of all the correlations - not sure what this
                % does, but it looks like it sums the CHANGE in correlations
                % between peaks to determine strength of the peaks.
                % sum_total = abs(correlation(find_total(2:end-1))-correlation(find_total(1:end-2)))+ abs(correlation(find_total(2:end-1))-correlation(find_total(3:end)));

                % Sort the peaks
                extrema_correlation = correlation_currentScan(find_total);
                [~,sort_locs] = sort(extrema_correlation,'descend');

                % Initialize the extrema matrix
                extrema_currentScan=zeros(size(correlation_currentScan));

                % Save the extrema locations with an index of 1
                if length(sort_locs) > final_peaks
                    extrema_currentScan(find_total(sort_locs(1:final_peaks)))=1;
                else
                    extrema_currentScan(find_total(sort_locs))=1;
                end

                extrema(:,idx_scan) = extrema_currentScan;
                correlation(:,idx_scan) = correlation_currentScan;
            end
        end
        
    end
    
end