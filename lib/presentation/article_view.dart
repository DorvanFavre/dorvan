import 'package:dorvan/application/constants/theme.dart';
import 'package:dorvan/application/use_cases/precache_next_image.dart';
import 'package:dorvan/domain/entity/article.dart';
import 'package:dorvan/presentation/rive_animation.dart';
import 'package:flutter/material.dart';

import 'aware_text.dart';

class ArticleView extends StatelessWidget {
  final Article? article;

  ArticleView({@required this.article});

  @override
  Widget build(BuildContext context) {
    return Stack(
      children: [
        Positioned(
          left: -50,
          top: 0,
          child: Text(
            article?.number ?? '-',
            maxLines: 1,
            softWrap: false,
            overflow: TextOverflow.visible,
            style: TextStyle(
                color: Colors.white, fontSize: 250, fontFamily: 'Futura'),
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
                  article!.title,
                  builder: (text) => Text(
                    text,
                    style: TextStyle(color: kArticleTitleColor, fontSize: 30),
                  ),
                ),
                SizedBox(height: 20),
                AwareText(
                  article!.description,
                  builder: (text) => Text(
                    text,
                    style: TextStyle(
                        color: kArticleDescriptionColor, fontSize: 16),
                  ),
                ),
                // scroll
              ],
            ),
          ),
        ),
        /*FutureBuilder(
          future: Future.delayed(Duration(seconds: 3)),
          builder: (context, value) =>
              value.connectionState == ConnectionState.done
                  ? Align(
                      alignment: Alignment.bottomCenter,
                      child: Padding(
                        padding: const EdgeInsets.only(bottom: 40),
                        child: Container(
                          height: 50,
                          width: 50,
                          child: RiveAnimation(
                            animation: '006_scroll_grey.riv',
                          ),
                        ),
                      ))
                  : SizedBox.shrink(),
        )*/
      ],
    );
  }
}
