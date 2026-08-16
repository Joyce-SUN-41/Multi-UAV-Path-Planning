function [result, meta] = mu_load_result(arg, varargin)
% mu_load_result — 读取由 mu_save_result 保存的规划结果 .mat
%
% 用法:
%   [result, meta] = mu_load_result(matPath)                      读取单个 .mat
%   [result, meta] = mu_load_result('latest', outDir)             读取某目录下最新一份单用例 .mat
%   paths            = mu_load_result('list',   outDir)           列出目录内所有结果
%   runs             = mu_load_result('experiment', expName, outDir)  读取实验集合（返回 runs cell）
%   index            = mu_load_result('index',  outDir)           读取 results/index.json 可读清单
%
% 兼容性: terrainF 在保存时被替换为 terrainFInfo；load 端若发现缺失且存在 Info，
%         提示用 mu_city_layout 重建（保持只读、不擅自改数据）。

defaultDir = fullfile(fileparts(mfilename('fullpath')), 'results');
outDir = defaultDir;
mode = 'file';
expName = '';
if ischar(arg) && any(strcmpi(arg, {'latest','list','index','experiment'})), mode = lower(arg); end
if nargin >= 2
    if strcmpi(mode,'experiment')
        expName = varargin{1};
        if numel(varargin) >= 2, outDir = varargin{2}; end
    else
        outDir = varargin{1};
    end
end

if strcmpi(mode,'list')
    files = dir(fullfile(outDir, 'muav_*.mat'));
    result = fullfile(outDir, {files.name})';
    meta = struct('count', numel(files));
    return;
end

if strcmpi(mode,'index')
    idxFile = fullfile(outDir, 'index.json');
    if ~exist(idxFile,'file'), error('mu_load_result: 未找到 %s', idxFile); end
    result = jsondecode(fileread(idxFile));
    meta = struct('source', idxFile);
    return;
end

if strcmpi(mode,'experiment')
    % 调用形式: mu_load_result('experiment', expName, outDir)
    en = matlab.lang.makeValidName(expName);
    fname = fullfile(outDir, sprintf('exp_%s.mat', en));
    if ~exist(fname,'file'), error('mu_load_result: 未找到实验集合 %s', fname); end
    d = load(fname, 'runs', 'expMeta');
    result = d.runs;
    meta = d.expMeta;
    fprintf('  loaded experiment set: %s (%d runs)\n', fname, numel(d.runs));
    return;
end

if strcmpi(mode,'latest')
    files = dir(fullfile(outDir, 'muav_*.mat'));
    if isempty(files), error('mu_load_result: 目录 %s 下没有 muav_*.mat', outDir); end
    [~, idx] = max([files.datenum]);
    matPath = fullfile(outDir, files(idx).name);
else
    matPath = arg;
end

d = load(matPath, 'stored', 'meta');
result = d.stored;
meta   = d.meta;

if isfield(result,'scene') && ~isfield(result.scene,'terrainF') && isfield(result.scene,'terrainFInfo')
    warning('mu_load_result: terrainF was not saved (function handles are not serializable); terrainFInfo kept.\n  To rebuild the height field, call mu_city_layout(result.scene.difficulty, result.scene.seed) and assign it.');
end
fprintf('  loaded: %s  (schema v%s, %s)\n', matPath, meta.schemaVersion, meta.savedAtUTC);
end
