function mu_save_ui(ax, finalPath, fmt)
% mu_save_ui ????UIAxesexportgraphics
%   ax        : UIAxes 
%   finalPath : ??%   fmt       : 'png' | 'eps'
% ??"<name>.tmp<ext>"exportgraphics  movefile ??[dr, nm, ext] = fileparts(finalPath);
if isempty(dr), dr = '.'; end
tmpPath = fullfile(dr, [nm '.tmp' ext]);
if exist(tmpPath,'file'), delete(tmpPath); end
try
    if strcmpi(fmt,'eps')
        exportgraphics(ax, tmpPath, 'ContentType','vector', 'BackgroundColor','white');
    else
        exportgraphics(ax, tmpPath, 'Resolution',300, 'BackgroundColor','white');
    end
    movefile(tmpPath, finalPath, 'f');
catch ME
    if exist(tmpPath,'file'), delete(tmpPath); end
    rethrow(ME);
end
end
