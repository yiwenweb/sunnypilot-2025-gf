"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import time

import requests
from requests.exceptions import (SSLError, RequestException, HTTPError)
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from sunnypilot.models.helpers import is_bundle_version_compatible

from cereal import custom


class ModelParser:
  """Handles parsing of model data into cereal objects"""

  @staticmethod
  def _parse_download_uri(download_uri_data) -> custom.ModelManagerSP.DownloadUri:
    download_uri = custom.ModelManagerSP.DownloadUri()
    download_uri.uri = download_uri_data.get("url")
    download_uri.sha256 = download_uri_data.get("sha256")
    return download_uri

  @staticmethod
  def _parse_artifact(artifact_data) -> custom.ModelManagerSP.Artifact:
    artifact = custom.ModelManagerSP.Artifact()
    artifact.fileName = artifact_data.get("file_name")
    artifact.downloadUri = ModelParser._parse_download_uri(artifact_data.get("download_uri", {}))
    return artifact

  @staticmethod
  def _parse_model(model_data) -> custom.ModelManagerSP.Model:
    model = custom.ModelManagerSP.Model()

    model.type = model_data.get("type")
    model.artifact = ModelParser._parse_artifact(model_data.get("artifact", {}))
    if metadata := model_data.get("metadata"):
      model.metadata = ModelParser._parse_artifact(metadata)
    return model

  @staticmethod
  def _parse_overrides(overrides_data: dict[str, str]) -> list[custom.ModelManagerSP.Override]:
    overrides = []
    for key, value in overrides_data.items():
      override = custom.ModelManagerSP.Override()
      override.key = key
      override.value = value
      overrides.append(override)
    return overrides

  @staticmethod
  def _parse_bundle(bundle) -> custom.ModelManagerSP.ModelBundle:
    model_bundle = custom.ModelManagerSP.ModelBundle()
    model_bundle.index = int(bundle["index"])
    model_bundle.internalName = bundle["short_name"]
    model_bundle.displayName = bundle["display_name"]
    model_bundle.models = [ModelParser._parse_model(model) for model in bundle.get("models",[])]
    model_bundle.status = 0
    model_bundle.generation = int(bundle["generation"])
    model_bundle.environment = bundle["environment"]
    model_bundle.runner = bundle.get("runner", custom.ModelManagerSP.Runner.snpe)
    model_bundle.is20hz = bundle.get("is_20hz", False)
    model_bundle.minimumSelectorVersion = int(bundle["minimum_selector_version"])
    model_bundle.overrides = ModelParser._parse_overrides(bundle.get("overrides", {}))
    model_bundle.ref = bundle.get("ref")

    return model_bundle

  @staticmethod
  def parse_models(json_data: dict) -> list[custom.ModelManagerSP.ModelBundle]:
    bundles_raw = json_data.get("bundles", [])
    found_bundles = [ModelParser._parse_bundle(bundle) for bundle in bundles_raw]

    compatible_bundles = []
    for bundle in found_bundles:
      is_compat = is_bundle_version_compatible(bundle.to_dict())
      if is_compat:
        compatible_bundles.append(bundle)
    return compatible_bundles


class ModelCache:
  """Handles caching of model data to avoid frequent remote fetches"""

  def __init__(self, params: Params, cache_timeout: int = int(3600 * 1e9)):
    self.params = params
    self.cache_timeout = cache_timeout
    self._LAST_SYNC_KEY = "ModelManager_LastSyncTime"
    self._CACHE_KEY = "ModelManager_ModelsCache"

  def _is_expired(self) -> bool:
    """Checks if the cache has expired"""
    current_time = int(time.monotonic() * 1e9)
    last_sync = self.params.get(self._LAST_SYNC_KEY) or 0
    return bool(last_sync == 0) or (current_time - last_sync) >= self.cache_timeout

  def get(self) -> tuple[dict, bool]:
    """
    Retrieves cached model data and expiration status atomically.
    Returns: Tuple of (cached_data, is_expired)
    If no cached data exists or on error, returns an empty dict
    """
    try:
      cached_data = self.params.get(self._CACHE_KEY)
      if not cached_data:
        cloudlog.warning("No cached model data available")
        return {}, True
      return cached_data, self._is_expired()
    except Exception as e:
      cloudlog.exception(f"Error retrieving cached model data: {str(e)}")
      return {}, True

  def set(self, data: dict) -> None:
    """Updates the cache with new model data"""
    self.params.put(self._CACHE_KEY, data)
    self.params.put(self._LAST_SYNC_KEY, int(time.monotonic() * 1e9))


class ModelFetcher:
  """Handles fetching and caching of model data from remote source"""
  # 官方模型配置 URL (直接使用 GitHub raw URL 避免 301 重定向延迟)
  MODEL_URL = "https://op.berrysoft.net/data/storage/driving_models_v16.json"
  # 本地自定义模型配置路径 (设置为 None 禁用自定义模型)
  CUSTOM_JSON_PATH = "/data/openpilot/sunnypilot/models/customer/my_models.json"

  def __init__(self, params: Params):
    self.params = params
    self.model_cache = ModelCache(params)
    self.model_parser = ModelParser()

  def _load_local_json(self, file_path: str) -> list[custom.ModelManagerSP.ModelBundle] | None:
    """从本地 JSON 文件加载模型配置（不影响官方模型缓存）"""
    import json
    try:
      with open(file_path, 'r', encoding='utf-8') as f:
        json_data = json.load(f)
      # 注意：不要设置缓存，避免覆盖官方模型数据
      # self.model_cache.set(json_data)  # 已移除
      cloudlog.info(f"成功加载本地模型配置: {file_path}")
      return self.model_parser.parse_models(json_data)
    except FileNotFoundError:
      cloudlog.error(f"本地模型配置文件不存在: {file_path}")
    except json.JSONDecodeError as e:
      cloudlog.error(f"本地模型配置 JSON 解析失败: {e}")
    except Exception as e:
      cloudlog.exception(f"加载本地模型配置时发生错误: {e}")
    return None

  def _fetch_and_cache_models(self) -> list[custom.ModelManagerSP.ModelBundle] | None:
    """Fetches fresh model data from remote and updates cache.
    Returns None on transport errors. Raises on 404 and other fatal HTTP errors.
    支持本地文件: 使用 file:// 前缀
    """
    # 检查是否为本地文件路径
    if self.MODEL_URL.startswith("file://"):
      local_path = self.MODEL_URL[7:]  # 去掉 "file://" 前缀
      return self._load_local_json(local_path)

    try:
      response = requests.get(self.MODEL_URL, timeout=60)

      # Explicitly handle 404 differently
      if response.status_code == 404:
        cloudlog.error(f"Models URL returned 404 Not Found: {self.MODEL_URL}")
        raise HTTPError(f"404 Not Found: {self.MODEL_URL}", response=response)

      # Raise for any other 4xx/5xx
      response.raise_for_status()

      json_data = response.json()
      self.model_cache.set(json_data)
      return self.model_parser.parse_models(json_data)

    except Exception as e:
      cloudlog.exception(f"Unexpected error fetching models: {e}")

    return None

  def get_available_bundles(self) -> list[custom.ModelManagerSP.ModelBundle]:
    """Gets the list of available models, with smart cache handling
    合并官方模型和本地自定义模型
    """
    all_bundles = []

    # 1. 获取官方模型
    cached_data, is_expired = self.model_cache.get()

    if cached_data and not is_expired:
      all_bundles = self.model_parser.parse_models(cached_data)
    else:
      fetched_bundles = self._fetch_and_cache_models()
      if fetched_bundles is not None:
        all_bundles = fetched_bundles
      elif cached_data:
        all_bundles = self.model_parser.parse_models(cached_data)

    # 2. 加载本地自定义模型并合并
    if self.CUSTOM_JSON_PATH:
      custom_bundles = self._load_local_json(self.CUSTOM_JSON_PATH)
      if custom_bundles:
        # 获取官方模型的 index 列表，避免重复
        existing_indexes = {b.index for b in all_bundles}
        for bundle in custom_bundles:
          if bundle.index not in existing_indexes:
            all_bundles.append(bundle)
            cloudlog.info(f"已添加自定义模型: {bundle.displayName}")

    return all_bundles

if __name__ == "__main__":
  params = Params()
  model_fetcher = ModelFetcher(params)
  model_fetcher.get_available_bundles()
