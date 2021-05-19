import 'package:dorvan/application/providers/app_content_provider.dart';
import 'package:dorvan/application/providers/page_controller_provider.dart';
import 'package:dorvan/application/use_cases/precache_next_image.dart';
import 'package:dorvan/domain/entity/article.dart';
import 'package:dorvan/domain/entity/chapter.dart';
import 'package:dorvan/domain/entity/contact.dart';
import 'package:dorvan/domain/entity/intro.dart';
import 'package:dorvan/domain/entity/page_content.dart';
import 'package:dorvan/domain/entity/picture.dart';
import 'package:dorvan/presentation/chapter_view.dart';
import 'package:dorvan/presentation/chapter_view.dart';
import 'package:dorvan/presentation/contact_view.dart';
import 'package:dorvan/presentation/picture_view.dart';
import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import 'intro_view.dart';

class MainScrollView extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.black,
      body: PageView.builder(
        controller: context.read(pageControllerProvider),
        scrollDirection: Axis.vertical,
        itemCount: context.read(appContentProvider).length,
        itemBuilder: (context, index) {
          final pageContent = context.read(appContentProvider)[index];

          // Precache image
          if (index + 1 < context.read(appContentProvider).length) {
            PrecacheNextImage(
                context, context.read(appContentProvider)[index + 1]);
          }

          if (pageContent is Intro) {
            return IntroView(
              intro: pageContent,
            );
          } else if (pageContent is Chapter) {
            return ChapterView(chapter: pageContent);
          } else if (pageContent is Picture) {
            return PictureView(
              picture: pageContent,
            );
          } else if (pageContent is Contact) {
            return ContactView();
          }
          return SizedBox.shrink();
        },
      ),
    );
  }
}
