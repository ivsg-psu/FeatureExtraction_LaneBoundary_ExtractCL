function fcn_ExtractCL_plotModelVsData(SegTable, fig_num, varargin)
% Overlay raw XY points and fitted XY models per segment.
p = inputParser;
addParameter(p,'Nsamp',300,@(x)isnumeric(x)&&isscalar(x));
addParameter(p,'ShowPoints',true,@islogical);
addParameter(p, 'ModelType', 'XY');
parse(p,varargin{:});
opt = p.Results;

figure(fig_num);
clf;
hold on; 
axis equal; 
grid on;
grid minor;
box on;

col = containers.Map({'line','arc','spiral','poly3'}, ...
    {[0.0000 0.4470 0.7410], ...  % blue
     [0.8500 0.3250 0.0980], ...  % orange
     [0.4940 0.1840 0.5560], ...  % purple
     [0.0000 0.3900 0.0000]});    % dark green

legend_added = containers.Map('KeyType','char','ValueType','logical');

for i=1:height(SegTable)
    d = SegTable.SegmentData{i};
    X=d(:,1); Y=d(:,2); S=d(:,4);
    [S,ord] = sort(S); 
    X=X(ord); 
    Y=Y(ord);
    if strcmpi(opt.ModelType, 'XY')
        mdl = SegTable.XYModel{i};
    elseif strcmpi(opt.ModelType, 'Smooth')
        mdl = SegTable.Model_Smooth{i};
    else
        mdl = SegTable.XYModel{i}; % fallback
    end

    Sr  = mdl.SRange;
    Sq  = linspace(Sr(1), Sr(2), opt.Nsamp).';
    XYm = cell2mat(arrayfun(@(S) mdl.EvalXY(S), Sq, 'UniformOutput', false));
    XYm = reshape(XYm, [], 2);

    c = [0.4 0.4 0.4];
    try
        lab= char(SegTable.Label(i));
    catch
        lab= char(SegTable.Type(i));
    end
    if isKey(col,lab)
        c = col(lab); 
    end

    
    scatter(X,Y,50,c*0.7+0.3,'filled', 'HandleVisibility','off');
    % hold on
    if ~isKey(legend_added,lab) || ~legend_added(lab)
        plot(XYm(:,1), XYm(:,2), 'Color', c, 'LineWidth', 3, 'DisplayName', lab);
        legend_added(lab) = true;
    else
        plot(XYm(:,1), XYm(:,2), 'Color', c, 'LineWidth', 3, 'HandleVisibility','off');
    end
    % plot(XYm(:,1), XYm(:,2), 'Color', c, 'LineWidth', 4,'DisplayName',lab);
    scatter(XYm(1,1), XYm(1,2), 50, 'red','filled','HandleVisibility','off')
    pause(0.1)
end
xlabel('X-East (m)'); ylabel('Y-North (m)');
legend('Location','best')
% legend show
title('Fitted XY models vs raw ENU points');
end
