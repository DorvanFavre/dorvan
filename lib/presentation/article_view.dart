import 'package:dorvan/application/use_cases/precache_next_image.dart';
import 'package:dorvan/domain/entity/article.dart';
import 'package:flutter/material.dart';

import 'aware_text.dart';

class ArticleView extends StatelessWidget {
  final Article? article;

  ArticleView({@required this.article});

  @override
  Widget build(BuildContext context) {
    PrecacheNextImage(context, article!);

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
                    style: TextStyle(color: Colors.black87, fontSize: 30),
                  ),
                ),
                SizedBox(height: 20),
                AwareText(
                  article!.description,
                  builder: (text) => Text(
                    text,
                    style: TextStyle(color: Colors.grey[600], fontSize: 16),
                  ),
                ),
              ],
            ),
          ),
        ),
      ],
    );
  }
}
