function mu_savefig(fig, finalPath, fmt, res)
% mu_savefig ????figure
%   fig       : figure 
%   finalPath : ??%   fmt       : 'png' | 'eps'
%   res       :  300??% ??"<name>.tmp<ext>" print  movefile ??%       ??if nargin < 4, res = 300; end
if nargin < 3 || isempty(fmt), fmt = 'png'; end

[dr, nm, ext] = fileparts(finalPath);
if isempty(dr), dr = '.'; end
tmpPath = fullfile(dr, [nm '.tmp' ext]);
if exist(tmpPath,'file'), delete(tmpPath); end

try
    if strcmpi(fmt,'eps')
        set(fig,'Renderer','painters');        %  painters
        print(fig, tmpPath, '-depsc', sprintf('-r%d',res));
    else
        print(fig, tmpPath, '-dpng', sprintf('-r%d',res));
    end
    % ??    movefile(tmpPath, finalPath, 'f');
catch ME
    if exist(tmpPath,'file'), delete(tmpPath); end
    rethrow(ME);
end
end
