import 'package:dorvan/domain/entity/page_content.dart';
import 'package:flutter/material.dart';

class Picture extends PageContent {
  final String image;
  final String title;
  final String description;

  Picture(this.image, this.title, this.description);
}
