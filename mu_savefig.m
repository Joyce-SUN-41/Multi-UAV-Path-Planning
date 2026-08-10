function mu_savefig(fig, finalPath, fmt, res)
% mu_savefig — 原子写保存 figure（防止进程被杀留下损坏的半截文件）
%   fig       : figure 句柄
%   finalPath : 最终文件路径（含扩展名）
%   fmt       : 'png' | 'eps'
%   res       : 分辨率（默认 300）
% 实现：先写 "<name>.tmp<ext>" 临时文件，print 成功后再 movefile 原子改名；
%       任何失败都删除临时文件，绝不留下损坏的正式文件。
if nargin < 4, res = 300; end
if nargin < 3 || isempty(fmt), fmt = 'png'; end

[dr, nm, ext] = fileparts(finalPath);
if isempty(dr), dr = '.'; end
tmpPath = fullfile(dr, [nm '.tmp' ext]);
if exist(tmpPath,'file'), delete(tmpPath); end

try
    if strcmpi(fmt,'eps')
        set(fig,'Renderer','painters');        % 矢量图必须用 painters
        print(fig, tmpPath, '-depsc', sprintf('-r%d',res));
    else
        print(fig, tmpPath, '-dpng', sprintf('-r%d',res));
    end
    % 原子改名（覆盖已存在文件）
    movefile(tmpPath, finalPath, 'f');
catch ME
    if exist(tmpPath,'file'), delete(tmpPath); end
    rethrow(ME);
end
end
