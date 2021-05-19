import 'dart:ui';

import 'package:dorvan/application/constants/app_infos.dart';
import 'package:dorvan/application/providers/app_content_provider.dart';
import 'package:dorvan/domain/entity/page_content.dart';
import 'package:dorvan/domain/entity/picture.dart';
import 'package:dorvan/presentation/responsive_image.dart';
import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

class PrecacheNextImage {
  final PageContent nextPageContent;

  PrecacheNextImage(BuildContext context, this.nextPageContent) {
    /*final appContent = context.read(appContentProvider);
    final nextIndex = appContent.indexOf(currentPageContent) + 1;
    print('next index : $nextIndex');
    if (nextIndex < appContent.length) {*/
    final nextfPageContent = nextPageContent;
    if (nextfPageContent is Picture) {
      final rawImage = nextfPageContent.image;
      final image = rawImage.substring(0, rawImage.length - 4);
      final node = rawImage.substring(rawImage.length - 4, rawImage.length);
      /*final responsiveImage = image +
          (window.physicalSize.aspectRatio > 1 ? '_landscape' : '_portrait') +
          node;
      precacheImage(AssetImage(kImagePath + responsiveImage), context);*/
      final responsiveImagePortrait = image + '_portrait' + node;
      final responsiveImageLandscape = image + '_landscape' + node;
      precacheImage(AssetImage(kImagePath + responsiveImagePortrait), context);
      precacheImage(AssetImage(kImagePath + responsiveImageLandscape), context);
    }
    // }
  }
}
