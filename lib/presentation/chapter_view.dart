import 'package:dorvan/application/constants/theme.dart';
import 'package:dorvan/application/providers/app_content_provider.dart';
import 'package:dorvan/application/use_cases/precache_next_image.dart';
import 'package:dorvan/domain/entity/article.dart';
import 'package:dorvan/domain/entity/chapter.dart';
import 'package:dorvan/domain/entity/picture.dart';
import 'package:dorvan/presentation/white_button.dart';
import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import 'aware_text.dart';
import 'second_scroll_view.dart';

class ChapterView extends StatelessWidget {
  final Chapter? chapter;

  ChapterView({@required this.chapter});
  @override
  Widget build(BuildContext context) {
    return Stack(
      children: [
        Positioned(
          left: -50,
          top: 0,
          child: Text(
            chapter?.chapter ?? '-',
            maxLines: 1,
            softWrap: false,
            overflow: TextOverflow.visible,
            style: TextStyle(
                color: kChapterNumberColor,
                fontSize: 250,
                fontFamily: 'Futura'),
          ),
        ),
        Padding(
          padding: const EdgeInsets.all(30.0),
          child: Align(
            alignment: Alignment(-1, 0.5),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                AwareText(
                  chapter!.title,
                  builder: (text) => Text(
                    text,
                    style: TextStyle(color: Colors.white, fontSize: 30),
                  ),
                ),
                SizedBox(height: 20),
                AwareText(
                  chapter!.description,
                  builder: (text) => Text(
                    text,
                    style: TextStyle(
                        color: kChapterDescriptionColor, fontSize: 16),
                  ),
                ),
                SizedBox(height: 30),
                WhiteButton(
                    onTap: () {
                      Navigator.of(context).push(MaterialPageRoute(
                          builder: (context) =>
                              SecondScrollView(chapter: chapter)));
                    },
                    text: 'button_chapter')
              ],
            ),
          ),
        ),
      ],
    );
  }
}
