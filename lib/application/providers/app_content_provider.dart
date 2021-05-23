import 'package:dorvan/domain/entity/article.dart';
import 'package:dorvan/domain/entity/chapter.dart';
import 'package:dorvan/domain/entity/contact.dart';
import 'package:dorvan/domain/entity/exit_page.dart';
import 'package:dorvan/domain/entity/intro.dart';
import 'package:dorvan/domain/entity/page_content.dart';
import 'package:dorvan/domain/entity/picture.dart';

import 'package:flutter_riverpod/flutter_riverpod.dart';
import 'package:flutter/material.dart';

final appContentProvider = Provider<List<PageContent>>((ref) {
  return [
    Intro(
      "chapter_intro_title",
      "chapter_intro_description",
      '00',
    ),
    Picture('dorvan.jpg', 'image_dorvan_title', 'image_dorvan_description'),
    // Entrepreneur
    Chapter('chapter_entrepreneur_title', "chapter_entrepreneur_description",
        '01', [
      Article('article_entrepreneur_title', 'article_entrepreneur_description',
          '01'),
      Picture('business_card.jpg', 'image_business_card_title',
          'image_business_card_description'),
      Article('article_parlezmoi_title', 'article_parlezmoi_description', '01'),
      Picture('logo.jpg', 'image_logo_title',
          'image_logo_description'),
      Article(
          'article_tirerprofit_title', 'article_tirerprofit_description', '01'),
      Article(
          'article_boutenbout_title', 'article_boutenbout_description', '01'),
      ExitPage('01'),
    ]),
    // Architect
    Chapter('chapter_architect_title', "chapter_architect_description", '02', [
      Article('article_architect_title', 'article_architect_description', '02'),
      Picture('uikit_haveadrink.jpg', 'image_haveadrink_title',
          'image_haveadrink_description'),
      Article('article_nature_title', 'article_nature_description', '02'),
      Article('article_pwa_title', 'article_pwa_description', '02'),
      ExitPage('02'),
    ]),
    // Designer
    Chapter('chapter_designer_title', "chapter_designer_description", '03', [
      Article('article_designer_title', 'article_designer_description', '03'),
      Picture('nike.jpg', 'image_nike_title', 'image_nike_description'),
      Article('article_artetweb_title', 'article_artetweb_description', '03'),
      Picture('stitch.jpg', 'image_stitch_title', 'image_stitch_description'),
      Article('article_premiereimpression_title',
          'article_premiereimpression_description', '03'),
      Article(
          'article_matechnique_title', 'article_matechnique_description', '03'),
      ExitPage('03'),
    ]),
    // Developer
    Chapter(
        'chapitre_developer_title', "chapitre_developer_description", '04', [
      Article('article_developer_title', 'article_developer_description', '04'),
      Picture('vscode.jpg', 'image_vscode_title', 'image_vscode_description'),
      Article(
          'article_technologie_title', 'article_technologie_description', '04'),
      Picture('snake.jpg', 'image_snake_title', 'image_snake_description'),
      Article('article_language_title', 'article_language_description', '04'),
      Article(
          'article_monsiteweb_title', 'article_monsiteweb_description', '04'),
      ExitPage('04'),
    ]),
    Contact('contact_title'),
  ];
});
