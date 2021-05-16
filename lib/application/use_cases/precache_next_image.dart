import 'package:dorvan/application/providers/app_content_provider.dart';
import 'package:dorvan/domain/entity/page_content.dart';
import 'package:dorvan/domain/entity/picture.dart';
import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

class PrecacheNextImage {
  final PageContent currentPageContent;

  PrecacheNextImage(BuildContext context, this.currentPageContent) {
    final appContent = context.read(appContentProvider);
    final nextIndex = appContent.indexOf(currentPageContent) + 1;
    if (nextIndex < appContent.length) {
      final nextPageContent = appContent[nextIndex];
      if (nextPageContent is Picture) {
        precacheImage(nextPageContent.imageAsset, context);
      }
    }
  }
}
