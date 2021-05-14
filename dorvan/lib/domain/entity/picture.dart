import 'package:dorvan/domain/entity/page_content.dart';
import 'package:flutter/material.dart';

class Picture extends PageContent {
  final String imageName;
  final String title;
  final String description;

  Picture(this.imageName, this.title, this.description);
}
