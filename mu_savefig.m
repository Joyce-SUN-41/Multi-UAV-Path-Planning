function mu_savefig(fig, finalPath, fmt, res)
% mu_savefig - robust figure export
%   fig       : figure handle
%   finalPath : final file path (extension added from fmt if absent)
%   fmt       : 'png' | 'eps'
%   res       : dpi (default 300)
% Design: print to a UNIQUE temp file, confirm it landed, then atomically
% rename. Retries once on headless-render hiccups. Never leaves the caller
% guessing with a cryptic "cannot read source" from movefile.

if nargin < 4, res = 300; end
if nargin < 3 || isempty(fmt), fmt = 'png'; end

[dr, nm, ext] = fileparts(finalPath);
if isempty(dr), dr = '.'; end
finalPath = fullfile(dr, [nm ext]);

% unique temp name (avoids any .tmp naming/leftover collision)
tmpPath = [tempname(dr) ext];
if exist(tmpPath,'file'), delete(tmpPath); end

% 设备名映射：print 要求 -d<device>（如 -dpng/-depsc），旧版 '-png' 非法
devMap = struct('png','dpng', 'eps','depsc');
dev = lower(fmt);
if isfield(devMap, dev), dev = devMap.(dev); end
doPrint = @(p) print(fig, p, sprintf('-%s', dev), sprintf('-r%d', res));
if strcmpi(fmt,'eps')
    set(fig,'Renderer','painters');   % vector renderer for eps
end

ok = false;
for attempt = 1:2
    try
        doPrint(tmpPath);
    catch ME
        if exist(tmpPath,'file'), delete(tmpPath); end
        if attempt == 2, rethrow(ME); end
        drawnow; continue;   % retry after a refresh
    end
    if exist(tmpPath,'file')
        ok = true; break;
    end
    if attempt == 2, break; end
    drawnow;   % print produced nothing; refresh and retry
end

if ~ok
    error('mu_savefig: print did not produce %s (headless render failed).', tmpPath);
end

if exist(finalPath,'file'), delete(finalPath); end
movefile(tmpPath, finalPath, 'f');
end
