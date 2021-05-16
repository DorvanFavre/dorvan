import 'package:dorvan/domain/entity/page_content.dart';
import 'package:flutter/material.dart';

class Picture extends PageContent {
  final AssetImage imageAsset;
  final String title;
  final String description;

  Picture(this.imageAsset, this.title, this.description);
}
